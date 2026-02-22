#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_FILE="umdloop_static_ip.service"
SERVICE_PATH="/etc/systemd/system/$SERVICE_FILE"

echo "Installing UMD Loop Static IP daemon..."

sudo cp "$SCRIPT_DIR/$SERVICE_FILE" "$SERVICE_PATH"
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_FILE"
sudo systemctl start "$SERVICE_FILE"

echo "Daemon installed and started successfully."
echo "Status:"
sudo systemctl status "$SERVICE_FILE" --no-pager
