#!/bin/bash
set -e

INET_IF="wlp5s0"
JETSON_IF="enx9cbf0d007947"

sudo sysctl -w net.ipv4.ip_forward=1

if ! grep -q '^net.ipv4.ip_forward=1$' /etc/sysctl.conf; then
  echo "net.ipv4.ip_forward=1" | sudo tee -a /etc/sysctl.conf
fi

sudo iptables -t nat -C POSTROUTING -o "$INET_IF" -j MASQUERADE 2>/dev/null || \
sudo iptables -t nat -A POSTROUTING -o "$INET_IF" -j MASQUERADE

sudo iptables -C FORWARD -i "$JETSON_IF" -o "$INET_IF" -j ACCEPT 2>/dev/null || \
sudo iptables -A FORWARD -i "$JETSON_IF" -o "$INET_IF" -j ACCEPT

sudo iptables -C FORWARD -i "$INET_IF" -o "$JETSON_IF" -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || \
sudo iptables -A FORWARD -i "$INET_IF" -o "$JETSON_IF" -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT

echo "Done. IP forwarding and NAT are configured."
