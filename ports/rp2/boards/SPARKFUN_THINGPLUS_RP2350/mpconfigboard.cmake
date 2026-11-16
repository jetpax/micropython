# cmake file for SparkFun Thing Plus RP2350

set(PICO_BOARD "sparkfun_thingplus_rp2350")
set(PICO_PLATFORM "rp2350")

# This DUT has W25Q128JV flash, but pico-sdk's sparkfun header picks
# boot2_w25q080 — the wrong dummy-cycle profile, which corrupts the XIP
# cache during boot2's QMI reconfig and crt0 hardfaults intermittently.
# Force the universal-but-slower generic 03h boot2.
set(PICO_DEFAULT_BOOT_STAGE2 boot2_generic_03h)

# Also skip MP's embedded boot2 invocation — bootrom's conservative XIP
# setup is enough, and skipping the boot2 transition window prevents the
# cache poisoning that breaks USB enumeration on this DUT.
set(MICROPY_HW_NO_EMBED_XIP_SETUP ON)

set(MICROPY_PY_LWIP ON)
set(MICROPY_PY_NETWORK_CYW43 ON)

# Bluetooth
set(MICROPY_PY_BLUETOOTH ON)
set(MICROPY_BLUETOOTH_BTSTACK ON)
set(MICROPY_PY_BLUETOOTH_CYW43 ON)

# Board specific version of the frozen manifest
set(MICROPY_FROZEN_MANIFEST ${MICROPY_BOARD_DIR}/manifest.py)
