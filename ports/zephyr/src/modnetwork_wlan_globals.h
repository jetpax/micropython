/* Module-globals fragment included into extmod/modnetwork.c via
 * MICROPY_PY_NETWORK_MODULE_GLOBALS_INCLUDEFILE. Adds the WLAN class and
 * the conventional STA_IF/AP_IF integer constants to network.*.
 */
#if defined(CONFIG_NET_L2_WIFI_MGMT)
{ MP_ROM_QSTR(MP_QSTR_WLAN), MP_ROM_PTR(&network_wlan_type) },
#endif
{ MP_ROM_QSTR(MP_QSTR_STA_IF), MP_ROM_INT(MOD_NETWORK_STA_IF) },
{ MP_ROM_QSTR(MP_QSTR_AP_IF), MP_ROM_INT(MOD_NETWORK_AP_IF) },
