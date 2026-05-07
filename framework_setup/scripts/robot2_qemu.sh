#!/usr/bin/env bash
# Boot robot2 QEMU VM.
# Set DISK to override the default image path.
DISK="${DISK:-$HOME/qemu_setup/robot2.img}"

qemu-system-x86_64 \
  -enable-kvm \
  -cpu host \
  -boot menu=on \
  -m 4G -smp 4 \
  -drive file="$DISK",if=virtio,cache=writeback,format=qcow2 \
  -netdev user,id=ssh0,hostfwd=tcp::2223-:22 \
  -device virtio-net-pci,netdev=ssh0 \
  -netdev tap,id=rosnet,ifname=tap1,script=no,downscript=no \
  -device virtio-net-pci,netdev=rosnet,mac=52:54:00:60:00:02
