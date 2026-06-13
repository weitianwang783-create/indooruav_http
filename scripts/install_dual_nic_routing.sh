#!/usr/bin/env bash

set -euo pipefail

if [[ $(id -u) -ne 0 ]]; then
  echo "ERROR: must run as root: sudo bash $0"
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_DIR=/opt/indooruav/scripts
ENV_DIR=/etc/indooruav

# ---- Remove old radar-static-ip service ----
echo "Removing old radar-static-ip service..."
systemctl stop radar-static-ip.service 2>/dev/null || true
systemctl disable radar-static-ip.service 2>/dev/null || true
rm -f /etc/systemd/system/radar-static-ip.service
rm -f "$ENV_DIR/radar-static-ip.env"
rm -f "$SCRIPT_DIR/configure_radar_static_ip.sh"

# ---- Install dual NIC routing service ----
echo "Installing dual NIC routing service..."
mkdir -p "$SCRIPT_DIR" "$ENV_DIR"

cp -f "$REPO_ROOT/scripts/configure_dual_nic_routing.sh" "$SCRIPT_DIR/"
chmod +x "$SCRIPT_DIR/configure_dual_nic_routing.sh"
cp -f "$REPO_ROOT/config/dual_nic_routing.env" "$ENV_DIR/"
cp -f "$REPO_ROOT/config/dual_nic_routing.service" /etc/systemd/system/

systemctl daemon-reload
systemctl enable dual_nic_routing.service

# ---- Clean up workspace static-ip files ----
rm -f "$REPO_ROOT/config/radar-static-ip.env"
rm -f "$REPO_ROOT/config/radar-static-ip.service"
rm -f "$REPO_ROOT/scripts/configure_radar_static_ip.sh"
rm -f "$REPO_ROOT/scripts/install_radar_static_ip.sh"

echo
echo "Done."
echo "  Service : /etc/systemd/system/dual_nic_routing.service"
echo "  Env     : $ENV_DIR/dual_nic_routing.env"
echo "  Script  : $SCRIPT_DIR/configure_dual_nic_routing.sh"
echo
echo "Next step:"
echo "  sudo systemctl start dual_nic_routing.service"
echo "  sudo systemctl status dual_nic_routing.service"
