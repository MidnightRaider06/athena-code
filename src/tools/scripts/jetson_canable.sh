#!/bin/bash
sudo ip link set can1 up type can \
	bitrate 1000000 \
	sjw 10 \
	restart-ms 1000

sudo ifconfig can1 txqueuelen 1000
