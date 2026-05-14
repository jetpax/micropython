/* File-scope declarations included into extmod/modnetwork.c via
 * MICROPY_PY_NETWORK_INCLUDEFILE. Provides the WLAN type symbol so the
 * globals include can reference it.
 */
#if defined(CONFIG_NET_L2_WIFI_MGMT)
extern const mp_obj_type_t network_wlan_type;
#endif
