#!/usr/bin/env bash
# SSH into robot2 QEMU VM (port 2223).
# Default credentials: robot2 / @robot2
ssh -p 2223 robot2@localhost "$@"
