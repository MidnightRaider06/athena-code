#!/bin/bash
# SPECIFICALLY for orin nx, pinmux wrong for other devices
set -euo pipefail

# load can kernel modules
modprobe can
modprobe can_raw
modprobe mttcan

# https://docs.nvidia.com/jetson/archives/r35.3.1/DeveloperGuide/text/HR/ControllerAreaNetworkCan.html#jetson-platform-details
busybox devmem 0x0c303018 w 0xc458  # can0_din
busybox devmem 0x0c303010 w 0xc400  # can0_dout

ip link set can0 up type can \
    bitrate 1000000 \
    berr-reporting on \
    restart-ms 1000
