/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2026 jetpax
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "py/runtime.h"
#include "py/objlist.h"
#include "py/objtuple.h"
#include "py/objstr.h"
#include "py/mperrno.h"
#include "py/mphal.h"

#if MICROPY_PY_NETWORK && defined(CONFIG_NET_L2_WIFI_MGMT)

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi.h>
#include <zephyr/net/wifi_mgmt.h>
#if defined(CONFIG_NET_DHCPV4)
#include <zephyr/net/dhcpv4.h>
#endif
#if defined(CONFIG_NET_HOSTNAME_ENABLE)
#include <zephyr/net/hostname.h>
#endif
#if defined(CONFIG_DNS_RESOLVER)
#include <zephyr/net/dns_resolve.h>
#endif

#include "extmod/modnetwork.h"

/* Status constants mirror rp2/cyw43 numerics so MP scripts port between
 * the two CYW43439-bearing platforms without changes. */
enum {
    STAT_IDLE             = 0,
    STAT_CONNECTING       = 1,
    STAT_WRONG_PASSWORD   = -3,
    STAT_NO_AP_FOUND      = -2,
    STAT_CONNECT_FAIL     = -1,
    STAT_GOT_IP           = 3,
};

typedef struct _network_wlan_obj_t {
    mp_obj_base_t base;
    int itf;  /* MOD_NETWORK_STA_IF or MOD_NETWORK_AP_IF */
    struct net_if *iface;
} network_wlan_obj_t;

extern const mp_obj_type_t network_wlan_type;

static network_wlan_obj_t network_wlan_sta_obj = {
    .base = { &network_wlan_type },
    .itf = MOD_NETWORK_STA_IF,
    .iface = NULL,
};

static struct net_if *resolve_iface(network_wlan_obj_t *self) {
    if (self->iface == NULL) {
        self->iface = net_if_get_first_wifi();
    }
    return self->iface;
}

static struct net_if *require_iface(network_wlan_obj_t *self) {
    struct net_if *iface = resolve_iface(self);
    if (iface == NULL) {
        mp_raise_OSError(MP_ENODEV);
    }
    return iface;
}

/* Push MP's cross-port hostname buffer into Zephyr so DHCP DISCOVER /
 * REQUEST advertises it as Option 12. Called from SYS_INIT (boot
 * default), wlan.connect() (latest user-set value picked up just
 * before joining), and wlan.config(hostname=...).
 */
static void apply_hostname_to_zephyr(void) {
#if defined(CONFIG_NET_HOSTNAME_DYNAMIC)
    size_t len = strlen(mod_network_hostname_data);
    if (len > 0) {
        (void)net_hostname_set(mod_network_hostname_data, len);
    }
#endif
}

/* Walk Zephyr's default DNS resolver context for the first IPv4 server. */
static bool first_ipv4_dns(struct net_in_addr *out) {
#if defined(CONFIG_DNS_RESOLVER)
    struct dns_resolve_context *ctx = dns_resolve_get_default();
    if (ctx == NULL) {
        return false;
    }
    for (size_t i = 0; i < ARRAY_SIZE(ctx->servers); i++) {
        const struct net_sockaddr *sa = &ctx->servers[i].dns_server;
        if (sa->sa_family == AF_INET) {
            const struct net_sockaddr_in *sin =
                (const struct net_sockaddr_in *)(const void *)sa;
            if (sin->sin_addr.s_addr != 0) {
                *out = sin->sin_addr;
                return true;
            }
        }
    }
#else
    (void)out;
#endif
    return false;
}

/* === scan: net_mgmt event callback collects results into a list ============ */

typedef struct {
    mp_obj_t list;
    bool done;
} wlan_scan_ctx_t;

static struct k_sem wlan_scan_sem;
static wlan_scan_ctx_t *wlan_scan_active = NULL;
static struct net_mgmt_event_callback wlan_scan_cb;

static void wlan_scan_handler(struct net_mgmt_event_callback *cb,
                              uint64_t mgmt_event, struct net_if *iface) {
    (void)iface;
    wlan_scan_ctx_t *ctx = wlan_scan_active;
    if (ctx == NULL) {
        return;
    }
    if (mgmt_event == NET_EVENT_WIFI_SCAN_RESULT) {
        const struct wifi_scan_result *res = (const struct wifi_scan_result *)cb->info;
        mp_obj_t entry[6];
        entry[0] = mp_obj_new_bytes(res->ssid, res->ssid_length);
        entry[1] = mp_obj_new_bytes(res->mac, res->mac_length);
        entry[2] = MP_OBJ_NEW_SMALL_INT(res->channel);
        entry[3] = MP_OBJ_NEW_SMALL_INT(res->rssi);
        entry[4] = MP_OBJ_NEW_SMALL_INT(res->security);
        entry[5] = mp_const_false;  /* hidden — Zephyr doesn't surface */
        mp_obj_list_append(ctx->list, mp_obj_new_tuple(6, entry));
    } else if (mgmt_event == NET_EVENT_WIFI_SCAN_DONE) {
        ctx->done = true;
        k_sem_give(&wlan_scan_sem);
    }
}

/* === methods =============================================================== */

static mp_obj_t network_wlan_make_new(const mp_obj_type_t *type, size_t n_args,
                                      size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 1, false);
    int itf = MOD_NETWORK_STA_IF;
    if (n_args == 1) {
        itf = mp_obj_get_int(args[0]);
    }
    if (itf == MOD_NETWORK_STA_IF) {
        return MP_OBJ_FROM_PTR(&network_wlan_sta_obj);
    }
    if (itf == MOD_NETWORK_AP_IF) {
        mp_raise_NotImplementedError(MP_ERROR_TEXT("AP mode"));
    }
    mp_raise_ValueError(MP_ERROR_TEXT("invalid interface"));
}

static void network_wlan_print(const mp_print_t *print, mp_obj_t self_in,
                               mp_print_kind_t kind) {
    (void)kind;
    network_wlan_obj_t *self = MP_OBJ_TO_PTR(self_in);
    struct net_if *iface = resolve_iface(self);
    const char *role = (self->itf == MOD_NETWORK_STA_IF) ? "STA" : "AP";
    const char *state = "no-iface";
    if (iface != NULL) {
        if (!net_if_is_admin_up(iface)) {
            state = "down";
        } else if (net_if_is_dormant(iface)) {
            state = "idle";
        } else {
            state = "up";
        }
    }
    mp_printf(print, "<WLAN %s %s>", role, state);
}

static mp_obj_t network_wlan_active(size_t n_args, const mp_obj_t *args) {
    network_wlan_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    struct net_if *iface = require_iface(self);
    if (n_args > 1) {
        bool up = mp_obj_is_true(args[1]);
        int ret = up ? net_if_up(iface) : net_if_down(iface);
        if (ret < 0 && ret != -EALREADY) {
            mp_raise_OSError(-ret);
        }
    }
    return mp_obj_new_bool(net_if_is_admin_up(iface));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_wlan_active_obj, 1, 2, network_wlan_active);

static mp_obj_t network_wlan_connect(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_ssid, ARG_key, ARG_bssid };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_ssid,  MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_key,   MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_bssid, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };
    network_wlan_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    struct net_if *iface = require_iface(self);

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args,
                     MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (args[ARG_ssid].u_obj == mp_const_none) {
        mp_raise_TypeError(MP_ERROR_TEXT("ssid required"));
    }
    size_t ssid_len;
    const char *ssid = mp_obj_str_get_data(args[ARG_ssid].u_obj, &ssid_len);
    if (ssid_len == 0 || ssid_len > WIFI_SSID_MAX_LEN) {
        mp_raise_ValueError(MP_ERROR_TEXT("ssid length"));
    }

    struct wifi_connect_req_params params;
    memset(&params, 0, sizeof(params));
    params.ssid = (const uint8_t *)ssid;
    params.ssid_length = ssid_len;
    params.channel = WIFI_CHANNEL_ANY;
    params.band = WIFI_FREQ_BAND_2_4_GHZ;
    params.mfp = WIFI_MFP_OPTIONAL;

    if (args[ARG_key].u_obj == mp_const_none) {
        params.security = WIFI_SECURITY_TYPE_NONE;
    } else {
        size_t key_len;
        const char *key = mp_obj_str_get_data(args[ARG_key].u_obj, &key_len);
        if (key_len < 8 || key_len > WIFI_PSK_MAX_LEN) {
            mp_raise_ValueError(MP_ERROR_TEXT("psk length"));
        }
        params.psk = (const uint8_t *)key;
        params.psk_length = key_len;
        params.security = WIFI_SECURITY_TYPE_PSK;
    }

    if (args[ARG_bssid].u_obj != mp_const_none) {
        mp_buffer_info_t b;
        mp_get_buffer_raise(args[ARG_bssid].u_obj, &b, MP_BUFFER_READ);
        if (b.len != WIFI_MAC_ADDR_LEN) {
            mp_raise_ValueError(MP_ERROR_TEXT("bssid length"));
        }
        memcpy(params.bssid, b.buf, WIFI_MAC_ADDR_LEN);
    }

    if (!net_if_is_admin_up(iface)) {
        int ret = net_if_up(iface);
        if (ret < 0 && ret != -EALREADY) {
            mp_raise_OSError(-ret);
        }
    }

    /* Sync the latest hostname into Zephyr right before joining, so the
     * DHCP DISCOVER that fires post-link reflects any change the user
     * made via network.hostname(...) since boot.
     */
    apply_hostname_to_zephyr();

    int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
    if (ret < 0) {
        mp_raise_OSError(-ret);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(network_wlan_connect_obj, 1, network_wlan_connect);

static mp_obj_t network_wlan_disconnect(mp_obj_t self_in) {
    network_wlan_obj_t *self = MP_OBJ_TO_PTR(self_in);
    struct net_if *iface = require_iface(self);
    int ret = net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0);
    if (ret < 0 && ret != -EALREADY) {
        mp_raise_OSError(-ret);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(network_wlan_disconnect_obj, network_wlan_disconnect);

static bool iface_has_ipv4(struct net_if *iface) {
    struct net_in_addr *addr = net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED);
    return addr != NULL && addr->s_addr != 0;
}

static mp_obj_t network_wlan_isconnected(mp_obj_t self_in) {
    network_wlan_obj_t *self = MP_OBJ_TO_PTR(self_in);
    struct net_if *iface = resolve_iface(self);
    if (iface == NULL) {
        return mp_const_false;
    }
    /* The wifi link is "up" once the driver clears the dormant flag (which
     * the brcmfmac driver does on WLC_E_LINK). Use oper_state directly rather
     * than NET_REQUEST_WIFI_IFACE_STATUS — works against any Zephyr wifi
     * driver and doesn't depend on iface_status being implemented.
     */
    if (net_if_oper_state(iface) != NET_IF_OPER_UP) {
        return mp_const_false;
    }
    return mp_obj_new_bool(iface_has_ipv4(iface));
}
static MP_DEFINE_CONST_FUN_OBJ_1(network_wlan_isconnected_obj, network_wlan_isconnected);

static mp_obj_t network_wlan_status(size_t n_args, const mp_obj_t *args) {
    network_wlan_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    struct net_if *iface = require_iface(self);
    struct wifi_iface_status st;
    memset(&st, 0, sizeof(st));
    int ret = net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &st, sizeof(st));
    if (ret < 0) {
        mp_raise_OSError(-ret);
    }
    if (n_args == 1) {
        switch (st.state) {
            case WIFI_STATE_DISCONNECTED:
            case WIFI_STATE_INACTIVE:
            case WIFI_STATE_INTERFACE_DISABLED:
                return MP_OBJ_NEW_SMALL_INT(STAT_IDLE);
            case WIFI_STATE_SCANNING:
            case WIFI_STATE_AUTHENTICATING:
            case WIFI_STATE_ASSOCIATING:
            case WIFI_STATE_ASSOCIATED:
            case WIFI_STATE_4WAY_HANDSHAKE:
            case WIFI_STATE_GROUP_HANDSHAKE:
                return MP_OBJ_NEW_SMALL_INT(STAT_CONNECTING);
            case WIFI_STATE_COMPLETED:
                return MP_OBJ_NEW_SMALL_INT(iface_has_ipv4(iface) ? STAT_GOT_IP : STAT_CONNECTING);
            default:
                return MP_OBJ_NEW_SMALL_INT(STAT_CONNECT_FAIL);
        }
    }
    /* status('rssi') / status('stations') */
    qstr key = mp_obj_str_get_qstr(args[1]);
    if (key == MP_QSTR_rssi) {
        return MP_OBJ_NEW_SMALL_INT(st.rssi);
    }
    mp_raise_ValueError(MP_ERROR_TEXT("unsupported status key"));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_wlan_status_obj, 1, 2, network_wlan_status);

static mp_obj_t addr4_to_str(const struct net_in_addr *addr) {
    char buf[NET_IPV4_ADDR_LEN];
    if (net_addr_ntop(AF_INET, addr, buf, sizeof(buf)) == NULL) {
        return mp_obj_new_str_from_cstr("0.0.0.0");
    }
    return mp_obj_new_str_from_cstr(buf);
}

static mp_obj_t network_wlan_ifconfig(size_t n_args, const mp_obj_t *args) {
    network_wlan_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    struct net_if *iface = require_iface(self);

    if (n_args == 1) {
        struct net_in_addr ip = { 0 };
        struct net_in_addr nm = { 0 };
        struct net_in_addr gw = { 0 };
        struct net_in_addr dns = { 0 };
        struct net_in_addr *p = net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED);
        if (p != NULL) {
            ip = *p;
            nm = net_if_ipv4_get_netmask_by_addr(iface, p);
        }
        gw = net_if_ipv4_get_gw(iface);
        first_ipv4_dns(&dns);
        mp_obj_t tuple[4] = {
            addr4_to_str(&ip),
            addr4_to_str(&nm),
            addr4_to_str(&gw),
            addr4_to_str(&dns),
        };
        return mp_obj_new_tuple(4, tuple);
    }

    /* ifconfig('dhcp') — restart DHCP client. */
    if (mp_obj_is_str(args[1])) {
        const char *s = mp_obj_str_get_str(args[1]);
        if (strcmp(s, "dhcp") == 0) {
#if defined(CONFIG_NET_DHCPV4)
            net_dhcpv4_restart(iface);
            return mp_const_none;
#else
            mp_raise_NotImplementedError(MP_ERROR_TEXT("DHCPv4 disabled"));
#endif
        }
    }
    /* Static 4-tuple — punt; users typically rely on DHCP. */
    mp_raise_NotImplementedError(MP_ERROR_TEXT("static ifconfig"));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_wlan_ifconfig_obj, 1, 2, network_wlan_ifconfig);

static mp_obj_t network_wlan_scan(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    (void)n_args;
    (void)kw_args;
    network_wlan_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    struct net_if *iface = require_iface(self);

    if (!net_if_is_admin_up(iface)) {
        int ret = net_if_up(iface);
        if (ret < 0 && ret != -EALREADY) {
            mp_raise_OSError(-ret);
        }
    }

    if (wlan_scan_active != NULL) {
        mp_raise_OSError(MP_EBUSY);
    }

    wlan_scan_ctx_t ctx = { .list = mp_obj_new_list(0, NULL), .done = false };
    k_sem_reset(&wlan_scan_sem);
    wlan_scan_active = &ctx;
    net_mgmt_init_event_callback(&wlan_scan_cb, wlan_scan_handler,
                                 NET_EVENT_WIFI_SCAN_RESULT | NET_EVENT_WIFI_SCAN_DONE);
    net_mgmt_add_event_callback(&wlan_scan_cb);

    int ret = net_mgmt(NET_REQUEST_WIFI_SCAN, iface, NULL, 0);
    if (ret < 0) {
        net_mgmt_del_event_callback(&wlan_scan_cb);
        wlan_scan_active = NULL;
        mp_raise_OSError(-ret);
    }

    /* Drop the GIL so MP callbacks / RX thread keep running while we wait. */
    MP_THREAD_GIL_EXIT();
    k_sem_take(&wlan_scan_sem, K_SECONDS(15));
    MP_THREAD_GIL_ENTER();

    net_mgmt_del_event_callback(&wlan_scan_cb);
    wlan_scan_active = NULL;
    return ctx.list;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(network_wlan_scan_obj, 1, network_wlan_scan);

static mp_obj_t network_wlan_config(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    network_wlan_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    struct net_if *iface = require_iface(self);

    if (kwargs != NULL && kwargs->used != 0) {
        if (n_args != 1) {
            mp_raise_TypeError(MP_ERROR_TEXT("config: positional+keyword mix"));
        }
        for (size_t i = 0; i < kwargs->alloc; i++) {
            if (!mp_map_slot_is_filled(kwargs, i)) {
                continue;
            }
            /* hostname / country forward to the cross-port helpers (which
             * the user can also call directly as network.hostname / .country).
             */
            qstr key = mp_obj_str_get_qstr(kwargs->table[i].key);
            if (key == MP_QSTR_hostname) {
                mp_obj_t fa[1] = { kwargs->table[i].value };
                mod_network_hostname(1, fa);
                apply_hostname_to_zephyr();
                continue;
            }
            mp_raise_ValueError(MP_ERROR_TEXT("unsupported config key"));
        }
        return mp_const_none;
    }

    if (n_args != 2) {
        mp_raise_TypeError(MP_ERROR_TEXT("config: one query at a time"));
    }
    qstr key = mp_obj_str_get_qstr(args[1]);

    if (key == MP_QSTR_mac) {
        struct net_linkaddr *la = net_if_get_link_addr(iface);
        if (la == NULL || la->len == 0) {
            mp_raise_OSError(MP_ENODEV);
        }
        return mp_obj_new_bytes(la->addr, la->len);
    }
    if (key == MP_QSTR_hostname) {
        return mod_network_hostname(0, NULL);
    }

    /* All other queries need a live iface_status snapshot. */
    struct wifi_iface_status st;
    memset(&st, 0, sizeof(st));
    int ret = net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &st, sizeof(st));
    if (ret < 0) {
        mp_raise_OSError(-ret);
    }

    if (key == MP_QSTR_ssid || key == MP_QSTR_essid) {
        return mp_obj_new_bytes((const byte *)st.ssid, st.ssid_len);
    }
    if (key == MP_QSTR_channel) {
        return MP_OBJ_NEW_SMALL_INT(st.channel);
    }
    if (key == MP_QSTR_rssi) {
        return MP_OBJ_NEW_SMALL_INT(st.rssi);
    }
    mp_raise_ValueError(MP_ERROR_TEXT("unsupported config key"));
}
static MP_DEFINE_CONST_FUN_OBJ_KW(network_wlan_config_obj, 1, network_wlan_config);

/* === type ================================================================== */

static const mp_rom_map_elem_t network_wlan_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_active),      MP_ROM_PTR(&network_wlan_active_obj) },
    { MP_ROM_QSTR(MP_QSTR_connect),     MP_ROM_PTR(&network_wlan_connect_obj) },
    { MP_ROM_QSTR(MP_QSTR_disconnect),  MP_ROM_PTR(&network_wlan_disconnect_obj) },
    { MP_ROM_QSTR(MP_QSTR_isconnected), MP_ROM_PTR(&network_wlan_isconnected_obj) },
    { MP_ROM_QSTR(MP_QSTR_status),      MP_ROM_PTR(&network_wlan_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_ifconfig),    MP_ROM_PTR(&network_wlan_ifconfig_obj) },
    { MP_ROM_QSTR(MP_QSTR_scan),        MP_ROM_PTR(&network_wlan_scan_obj) },
    { MP_ROM_QSTR(MP_QSTR_config),      MP_ROM_PTR(&network_wlan_config_obj) },

    { MP_ROM_QSTR(MP_QSTR_IF_STA),      MP_ROM_INT(MOD_NETWORK_STA_IF) },
    { MP_ROM_QSTR(MP_QSTR_IF_AP),       MP_ROM_INT(MOD_NETWORK_AP_IF) },

    { MP_ROM_QSTR(MP_QSTR_STAT_IDLE),           MP_ROM_INT(STAT_IDLE) },
    { MP_ROM_QSTR(MP_QSTR_STAT_CONNECTING),     MP_ROM_INT(STAT_CONNECTING) },
    { MP_ROM_QSTR(MP_QSTR_STAT_WRONG_PASSWORD), MP_ROM_INT(STAT_WRONG_PASSWORD) },
    { MP_ROM_QSTR(MP_QSTR_STAT_NO_AP_FOUND),    MP_ROM_INT(STAT_NO_AP_FOUND) },
    { MP_ROM_QSTR(MP_QSTR_STAT_CONNECT_FAIL),   MP_ROM_INT(STAT_CONNECT_FAIL) },
    { MP_ROM_QSTR(MP_QSTR_STAT_GOT_IP),         MP_ROM_INT(STAT_GOT_IP) },
};
static MP_DEFINE_CONST_DICT(network_wlan_locals_dict, network_wlan_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    network_wlan_type,
    MP_QSTR_WLAN,
    MP_TYPE_FLAG_NONE,
    make_new, network_wlan_make_new,
    print, network_wlan_print,
    locals_dict, &network_wlan_locals_dict
    );

static int network_wlan_module_init(void) {
    k_sem_init(&wlan_scan_sem, 0, 1);
    /* Push the boot-time hostname default into Zephyr so the very first
     * DHCP DISCOVER (driver-triggered on WLC_E_LINK) carries it.
     */
    apply_hostname_to_zephyr();
    return 0;
}
SYS_INIT(network_wlan_module_init, APPLICATION, 90);

#endif /* MICROPY_PY_NETWORK && CONFIG_NET_L2_WIFI_MGMT */
