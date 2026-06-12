#!/usr/bin/env bash

set -euo pipefail

ETH_IFACE="eth0"
RADAR_HOST_IP="192.168.10.50"
RADAR_PREFIX_LEN="24"
RADAR_DEVICE_IP="192.168.10.3"

usage() {
  cat <<'EOF'
Usage:
  sudo bash scripts/configure_radar_static_ip.sh [options]

Options:
  --eth-iface <name>      Wired interface connected to the radar.
                          Default: eth0
  --host-ip <ipv4>        Static IPv4 address assigned to the host-side radar NIC.
                          Default: 192.168.10.50
  --prefix-len <bits>     Prefix length for the radar subnet.
                          Default: 24
  --radar-ip <ipv4>       Radar device IPv4 address.
                          Default: 192.168.10.3

What it does:
  1. Flushes IPv4 addresses from the radar NIC.
  2. Assigns a fixed static IPv4 address to the radar NIC.
  3. Adds a direct host route to the radar.

This avoids dual-NIC same-subnet conflicts by keeping the radar link on a
dedicated subnet, while Wi-Fi can continue using whatever DHCP network is
available in the current environment.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --eth-iface)
      ETH_IFACE="$2"
      shift 2
      ;;
    --host-ip)
      RADAR_HOST_IP="$2"
      shift 2
      ;;
    --prefix-len)
      RADAR_PREFIX_LEN="$2"
      shift 2
      ;;
    --radar-ip)
      RADAR_DEVICE_IP="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

RADAR_HOST_CIDR="${RADAR_HOST_IP}/${RADAR_PREFIX_LEN}"

echo "Configuring radar NIC:"
echo "  iface   : $ETH_IFACE"
echo "  host ip : $RADAR_HOST_CIDR"
echo "  radar ip: $RADAR_DEVICE_IP"

ip link set "$ETH_IFACE" up
ip addr flush dev "$ETH_IFACE" scope global
ip addr add "$RADAR_HOST_CIDR" dev "$ETH_IFACE"
ip route replace "${RADAR_DEVICE_IP}/32" dev "$ETH_IFACE" src "$RADAR_HOST_IP"

echo
echo "Applied address:"
ip -4 addr show dev "$ETH_IFACE"
echo
echo "Applied route:"
ip route get "$RADAR_DEVICE_IP" from "$RADAR_HOST_IP"
echo
echo "Next steps:"
echo "  1. Change the radar itself to the same subnet, e.g. $RADAR_DEVICE_IP/$RADAR_PREFIX_LEN."
echo "  2. Keep Wi-Fi on DHCP for normal LAN/front-end access."
echo "  3. Re-run this script after reboot unless you also persist it in NetworkManager/systemd."
