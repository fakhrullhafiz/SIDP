#!/usr/bin/env bash
set -euo pipefail

# uninstall_service.sh
# Usage: sudo ./uninstall_service.sh

if [ "$EUID" -ne 0 ]; then
  echo "This script must be run with sudo (or as root)." >&2
  exit 1
fi

SERVICE_NAME="sidp.service"
UNIT_PATH="/etc/systemd/system/$SERVICE_NAME"

echo "Stopping and disabling $SERVICE_NAME if present..."
if systemctl is-active --quiet "$SERVICE_NAME"; then
  systemctl stop "$SERVICE_NAME" || true
fi
if systemctl is-enabled --quiet "$SERVICE_NAME"; then
  systemctl disable "$SERVICE_NAME" || true
fi

if [ -f "$UNIT_PATH" ]; then
  echo "Removing unit file: $UNIT_PATH"
  rm -f "$UNIT_PATH"
fi

systemctl daemon-reload
echo "Uninstalled (if present)."
echo "You may also remove the project files from the device if desired."

exit 0
