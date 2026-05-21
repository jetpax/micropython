# Frozen boot module for the rpi_zero_2w board (scriptostudio / PiZZa).
#
# Mounts the microSD at the VFS root so the unified device-scripts tree
# (/boot.py, /main.py, /lib, /settings, /certs) resolves with the same
# absolute paths as the ESP and rp2 ports. The stock zephyr-port _boot.py
# mounts disks at /<name>; this board needs the SD as the root filesystem.
import sys
import os
import vfs
import zephyr


def _mount_sd_root():
    for name in zephyr.DiskAccess.disks:
        try:
            vfs.mount(zephyr.DiskAccess(name), "/")
            sys.path.append("/lib")
            os.chdir("/")
            return
        except OSError as e:
            print("_boot: failed to mount", name, "-", e)


_mount_sd_root()

del sys, os, vfs, zephyr, _mount_sd_root
