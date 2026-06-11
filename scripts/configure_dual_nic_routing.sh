#!/usr/bin/env bash

set -euo pipefail

WLAN_IFACE="wlan0"
ETH_IFACE="eth0"
RADAR_IP=""
WLAN_TABLE="100"
ETH_TABLE="200"
WLAN_RULE_PREF="10000"
ETH_RULE_PREF="10010"
WLAN_METRIC="100"
ETH_METRIC="900"

usage() {
  cat <<'EOF'
Usage:
  sudo bash scripts/configure_dual_nic_routing.sh [options]

Options:
  --wlan-iface <name>   Wireless interface used for developer/front-end access.
                        Default: wlan0
  --eth-iface <name>    Wired interface used for radar access.
                        Default: eth0
  --radar-ip <ip>       Radar IPv4 address. If omitted, the script tries to
                        infer one from "ip neigh show dev <eth-iface>".

What it does:
  1. Prefer the Wi-Fi NIC for the normal LAN subnet and default route.
  2. Pin the radar host route to the wired NIC.
  3. Add source-based policy routing so replies sourced from the Wi-Fi IP keep
     going out Wi-Fi even when both NICs are in the same subnet.
  4. Relax rp_filter and enable arp_filter to reduce same-subnet dual-NIC issues.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --wlan-iface)
      WLAN_IFACE="$2"
      shift 2
      ;;
    --eth-iface)
      ETH_IFACE="$2"
      shift 2
      ;;
    --radar-ip)
      RADAR_IP="$2"
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

require_ipv4_cidr() {
  local iface="$1"
  ip -o -4 addr show dev "$iface" primary scope global | awk '{print $4; exit}'
}

delete_rule_if_present() {
  local pref="$1"
  while ip rule show | grep -q "^${pref}:"; do
    ip rule del pref "$pref"
  done
}

WLAN_CIDR="$(require_ipv4_cidr "$WLAN_IFACE")"
ETH_CIDR="$(require_ipv4_cidr "$ETH_IFACE")"
WLAN_IP="${WLAN_CIDR%/*}"
ETH_IP="${ETH_CIDR%/*}"
WLAN_GW="$(ip route show default dev "$WLAN_IFACE" | awk '/default/ {print $3; exit}')"

if [[ -z "$RADAR_IP" ]]; then
  RADAR_IP="$(ip neigh show dev "$ETH_IFACE" | awk 'NR==1 {print $1}')"
fi

if [[ -z "$WLAN_CIDR" || -z "$ETH_CIDR" ]]; then
  echo "Failed to detect IPv4 addresses on $WLAN_IFACE or $ETH_IFACE." >&2
  exit 1
fi

if [[ -z "$WLAN_GW" ]]; then
  echo "Failed to detect a default gateway on $WLAN_IFACE." >&2
  exit 1
fi

echo "Wi-Fi : $WLAN_IFACE $WLAN_CIDR gateway $WLAN_GW"
echo "Wired : $ETH_IFACE $ETH_CIDR"
if [[ -n "$RADAR_IP" ]]; then
  echo "Radar : $RADAR_IP"
else
  echo "Radar : not detected, only generic policy routing will be configured"
fi

# Keep the main table biased toward Wi-Fi for the shared LAN subnet.
ip route replace "$WLAN_CIDR" dev "$WLAN_IFACE" scope link src "$WLAN_IP" metric "$WLAN_METRIC"
ip route replace "$ETH_CIDR" dev "$ETH_IFACE" scope link src "$ETH_IP" metric "$ETH_METRIC"
if [[ -n "$RADAR_IP" ]]; then
  ip route replace "${RADAR_IP}/32" dev "$ETH_IFACE" scope link src "$ETH_IP" metric 50
fi

# Rebuild policy routing tables.
ip route flush table "$WLAN_TABLE"
ip route add "$WLAN_CIDR" dev "$WLAN_IFACE" scope link src "$WLAN_IP" table "$WLAN_TABLE"
ip route add default via "$WLAN_GW" dev "$WLAN_IFACE" table "$WLAN_TABLE"

ip route flush table "$ETH_TABLE"
ip route add "$ETH_CIDR" dev "$ETH_IFACE" scope link src "$ETH_IP" table "$ETH_TABLE"
if [[ -n "$RADAR_IP" ]]; then
  ip route add "${RADAR_IP}/32" dev "$ETH_IFACE" scope link src "$ETH_IP" table "$ETH_TABLE"
fi

delete_rule_if_present "$WLAN_RULE_PREF"
delete_rule_if_present "$ETH_RULE_PREF"
ip rule add pref "$WLAN_RULE_PREF" from "${WLAN_IP}/32" table "$WLAN_TABLE"
ip rule add pref "$ETH_RULE_PREF" from "${ETH_IP}/32" table "$ETH_TABLE"

# Reduce ARP/rp_filter surprises when both NICs sit in the same subnet.
sysctl -w net.ipv4.conf.all.rp_filter=2 >/dev/null
sysctl -w "net.ipv4.conf.${WLAN_IFACE}.rp_filter=2" >/dev/null
sysctl -w "net.ipv4.conf.${ETH_IFACE}.rp_filter=2" >/dev/null
sysctl -w net.ipv4.conf.all.arp_filter=1 >/dev/null

echo
echo "Applied routes:"
ip route show
echo
echo "Applied rules:"
ip rule show
echo
echo "Quick checks:"
echo "  ip route get <developer_pc_ip>"
if [[ -n "$RADAR_IP" ]]; then
  echo "  ip route get $RADAR_IP"
fi
