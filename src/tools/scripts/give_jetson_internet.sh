#!/bin/bash
sudo sysctl -w net.ipv4.ip_forward=1
echo "net.ipv4.ip_forward=1" | sudo tee -a /etc/sysctl.conf

sudo iptables -t nat -A POSTROUTING -o wlp1s0 -j MASQUERADE

sudo iptables -A FORWARD -i eno1 -o wlp1s0 -j ACCEPT
sudo iptables -A FORWARD -i wlp1s0 -o eno1 -m state --state RELATED,ESTABLISHED -j ACCEPT

