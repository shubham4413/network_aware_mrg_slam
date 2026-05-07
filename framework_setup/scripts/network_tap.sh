#!/usr/bin/env bash
# Create bridge and tap interfaces for QEMU robot VMs.
# Run this on the host BEFORE booting the VMs.
set -euo pipefail

# --- tap0 for robot1 ---
sudo ip tuntap add dev tap0 mode tap user $USER
sudo ip addr add 192.168.60.1/24 dev tap0
sudo ip link set tap0 up

# --- tap1 for robot2 ---
sudo ip tuntap add dev tap1 mode tap user $USER
sudo ip addr add 192.168.61.1/24 dev tap1
sudo ip link set tap1 up

# --- Bridge both taps ---
sudo ip link add br0 type bridge
sudo ip link set tap0 master br0
sudo ip link set tap1 master br0
sudo ip link set br0 up

sudo ip addr add 192.168.60.1/24 dev br0
sudo ip addr flush dev tap0
sudo ip addr flush dev tap1

# --- Cyclone DDS buffer tuning ---
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.rmem_default=10485760

echo "[OK] Bridge br0 with tap0 + tap1 ready."
