#!/usr/bin/env bash

set -euo pipefail

if [[ $(id -u) -ne 0 ]]; then
  echo "ERROR: must run as root: sudo bash $0"
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_SRC="$REPO_ROOT/config/radar-static-ip.service"
ENV_SRC="$REPO_ROOT/config/radar-static-ip.env"
SCRIPT_SRC="$REPO_ROOT/scripts/configure_radar_static_ip.sh"

SERVICE_DEST=/etc/systemd/system/radar-static-ip.service
ENV_DIR=/etc/indooruav
ENV_DEST="$ENV_DIR/radar-static-ip.env"
SCRIPT_DIR=/opt/indooruav/scripts
SCRIPT_DEST="$SCRIPT_DIR/configure_radar_static_ip.sh"

mkdir -p "$ENV_DIR" "$SCRIPT_DIR"
cp -f "$SERVICE_SRC" "$SERVICE_DEST"
cp -f "$ENV_SRC" "$ENV_DEST"
cp -f "$SCRIPT_SRC" "$SCRIPT_DEST"
chmod +x "$SCRIPT_DEST"

systemctl daemon-reload
systemctl enable radar-static-ip.service

echo "Installed radar static IP startup service."
echo "  Service file: $SERVICE_DEST"
echo "  Env file   : $ENV_DEST"
echo "  Script file: $SCRIPT_DEST"
echo
cat <<'EOF'
下一步：
  sudo systemctl start radar-static-ip.service
  sudo systemctl status radar-static-ip.service
EOF
