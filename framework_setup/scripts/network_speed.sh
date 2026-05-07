#!/usr/bin/env bash
# Set up HTB traffic shaping on tap interfaces.
# VM-to-host traffic gets full speed; VM-to-VM traffic is shaped.
# The distance_bandwidth_shaper node dynamically updates the 1:20 class rate.
set -euo pipefail

# Clean old tc rules
sudo tc qdisc del dev tap0 root 2>/dev/null || true
sudo tc qdisc del dev tap1 root 2>/dev/null || true
sudo tc filter del dev tap0 parent 1: 2>/dev/null || true
sudo tc filter del dev tap1 parent 1: 2>/dev/null || true

# Add HTB root qdisc
sudo tc qdisc add dev tap0 root handle 1: htb default 10 r2q 50
sudo tc qdisc add dev tap1 root handle 1: htb default 10 r2q 50

# Default fast class (VM <-> Host traffic)
sudo tc class add dev tap0 parent 1: classid 1:10 htb rate 200mbit ceil 200mbit
sudo tc class add dev tap1 parent 1: classid 1:10 htb rate 200mbit ceil 200mbit

# Shaped class (VM <-> VM traffic), initial 2mbit, updated dynamically by the shaper node
sudo tc class add dev tap0 parent 1: classid 1:20 htb rate 2mbit ceil 2mbit
sudo tc class add dev tap1 parent 1: classid 1:20 htb rate 2mbit ceil 2mbit

# Filters: classify only VM-to-VM traffic into the shaped class

# VM2 -> VM1 traffic exits tap0
sudo tc filter add dev tap0 protocol ip parent 1: prio 10 u32 \
  match ip dst 192.168.60.2/32 \
  match ip src 192.168.60.3/32 \
  flowid 1:20

# VM1 -> VM2 traffic exits tap1
sudo tc filter add dev tap1 protocol ip parent 1: prio 10 u32 \
  match ip dst 192.168.60.3/32 \
  match ip src 192.168.60.2/32 \
  flowid 1:20

echo "[OK] HTB traffic shaping configured."
echo "     VM-Host: 200 Mbit/s  |  VM-VM: 2 Mbit/s (initial)"
