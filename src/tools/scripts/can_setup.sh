#!/bin/bash
sudo killall slcand
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo slcand -o -c -s8 /dev/ttyACM1 can0
sudo ip link set can0 up type can bitrate 1000000
sudo ifconfig can0 txqueuelen 1000
