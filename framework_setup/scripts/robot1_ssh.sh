#!/usr/bin/env bash
# SSH into robot1 QEMU VM (port 2222).
# Default credentials: robot1 / @robot1
ssh -p 2222 robot1@localhost "$@"
