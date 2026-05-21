# Frozen manifest for the rpi_zero_2w board (scriptostudio / PiZZa).

# asyncio: the device-scripts are async throughout -- frozen as on every
# other Zephyr board (cf. boards/mimxrt1020_evk/manifest.py).
include("$(MPY_DIR)/extmod/asyncio")

# ntptime: NTP wall-clock sync used by main.py (_quick_ntp / ntp_sync_task).
require("ntptime")

# Board-specific _boot.py: mounts the microSD at the VFS root (/), unlike
# the stock ports/zephyr/modules/_boot.py which mounts disks at /<name>.
freeze("$(PORT_DIR)/boards/rpi_zero_2w", "_boot.py")
