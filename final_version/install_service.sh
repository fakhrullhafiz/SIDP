#!/usr/bin/env bash
set -euo pipefail

# install_service.sh
# Usage: sudo ./install_service.sh [PROJECT_ROOT]
# If PROJECT_ROOT is omitted, the script uses the parent directory of this script as the project root.

if [ "$EUID" -ne 0 ]; then
  echo "This script must be run with sudo (or as root)." >&2
  echo "Example: sudo ./install_service.sh /home/pi/YourProjectPath" >&2
  exit 1
fi

PROJECT_ROOT="${1:-$(dirname "$(readlink -f "$0")")/..}"
PROJECT_ROOT="$(readlink -f "$PROJECT_ROOT")"
WORKDIR="$PROJECT_ROOT/final_version"
TEMPLATE="$PROJECT_ROOT/final_version/sidp.service.template"
SERVICE_NAME="sidp.service"
UNIT_PATH="/etc/systemd/system/$SERVICE_NAME"
RUN_USER="${SUDO_USER:-pi}"

if [ ! -d "$WORKDIR" ]; then
  echo "Error: expected project final_version directory at: $WORKDIR" >&2
  exit 2
fi

if [ ! -f "$TEMPLATE" ]; then
  echo "Error: service template not found at: $TEMPLATE" >&2
  exit 3
fi

echo "Installing SIDP service"
echo "Project root: $PROJECT_ROOT"
echo "Working dir:  $WORKDIR"
echo "Service unit: $UNIT_PATH"
echo "Service will run as user: $RUN_USER"

# Render template
sed "s|%RUN_USER%|$RUN_USER|g; s|%WORKDIR%|$WORKDIR|g" "$TEMPLATE" > "$UNIT_PATH"
chmod 644 "$UNIT_PATH"

echo "Reloading systemd daemon and enabling service..."
systemctl daemon-reload
systemctl enable "$SERVICE_NAME"
systemctl start "$SERVICE_NAME"

echo "Service installed and started. Check status with:" 
echo "  sudo systemctl status $SERVICE_NAME"
echo "To view logs: sudo journalctl -u $SERVICE_NAME -f"

exit 0
