# Framework Setup

Step-by-step instructions for setting up the bandwidth-aware multi-robot graph SLAM system.

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble (base installation)
- QEMU/KVM (`sudo apt install -y qemu-kvm qemu-utils virt-manager`)

## Option A: Docker (if you don't have Ubuntu 22.04 / ROS2 Humble)

Build and run everything in a container:

```bash
git clone --recurse-submodules https://usergit.vcs.mmi.rwth-aachen.de/max.hermans/research-project-shubham.git
cd research-project-shubham
docker build -t bandwidth_aware_mrg_slam -f docker/Dockerfile .
docker run -it --rm bandwidth_aware_mrg_slam
```

The Docker image includes all dependencies, custom patches, and a pre-built workspace. The rest of the steps below (QEMU, networking) still apply to the host machine.

## Option B: Native Install (Ubuntu 22.04 + ROS2 Humble)

### Step 1: Clone and Build

```bash
git clone --recurse-submodules https://usergit.vcs.mmi.rwth-aachen.de/max.hermans/research-project-shubham.git
cd research-project-shubham
./framework_setup/scripts/setup_mrg_slam.sh
```

The setup script will:
1. Initialize the Multi-Robot-Graph-SLAM git submodule
2. Install system dependencies (PCL, SuiteSparse, Geographic, etc.)
3. Import ROS2 workspace packages via `vcs` and install ROS dependencies
4. Copy custom C++ files (`adaptive_cloud_transmitter.hpp`, `mrg_slam_component.cpp`) into the submodule
5. Copy custom message definitions (`LinkQuality.msg`, modified `SlamStatus.msg`) into `mrg_slam_msgs`
6. Symlink `distance_bandwidth_shaper` into the workspace
7. Build everything with `colcon build --symlink-install --parallel-workers 1`

After building, source the workspace:
```bash
source Multi-Robot-Graph-SLAM/install/setup.bash
```

## Step 2: Create QEMU VM Images

Create two Ubuntu 22.04 VMs that will act as the robots:

```bash
mkdir -p ~/qemu_setup && cd ~/qemu_setup
wget https://releases.ubuntu.com/22.04/ubuntu-22.04.4-desktop-amd64.iso

# Create disk images (40G each)
qemu-img create -f qcow2 robot1.img 40G
qemu-img create -f qcow2 robot2.img 40G

# First boot (with installer ISO) for robot1
qemu-system-x86_64 \
  -enable-kvm -m 4096 -cpu host -smp 4 \
  -cdrom ubuntu-22.04.4-desktop-amd64.iso \
  -drive file=robot1.img,format=qcow2 \
  -boot d \
  -netdev user,id=net0 -device e1000,netdev=net0 \
  -display gtk

# Repeat for robot2 with robot2.img
```

During installation, create users:
- robot1 VM: user `robot1`, password `@robot1`
- robot2 VM: user `robot2`, password `@robot2`

## Step 3: Install ROS2 and MRG SLAM in the VMs

Boot each VM and install ROS2 Humble and the Multi-Robot-Graph-SLAM framework following the same process as Step 1 (clone repo, run setup script).

## Step 4: Network Configuration

### 4.1 Host: Create bridge and tap interfaces

Run **before** booting VMs:

```bash
./framework_setup/scripts/network_tap.sh
```

This creates `tap0`, `tap1`, and bridges them via `br0` with host IP `192.168.60.1/24`.

### 4.2 Boot the VMs

```bash
./framework_setup/scripts/robot1_qemu.sh
./framework_setup/scripts/robot2_qemu.sh
```

You can override the disk image path:
```bash
DISK=~/qemu_setup/robot1.img ./framework_setup/scripts/robot1_qemu.sh
```

### 4.3 Configure VM network interfaces

On **robot1** (inside VM):
```bash
sudo ip addr add 192.168.60.2/24 dev ens4
sudo ip link set ens4 up
```

On **robot2** (inside VM):
```bash
sudo ip addr add 192.168.60.3/24 dev ens4
sudo ip link set ens4 up
```

### 4.4 Verify connectivity

```bash
# From host
ping 192.168.60.2   # robot1
ping 192.168.60.3   # robot2

# From robot1
ping 192.168.60.1   # host

# From robot2
ping 192.168.60.1   # host
```

### 4.5 (Optional) Firewall: restrict VM-to-VM to TCP only

To simulate constrained inter-robot links with only TCP on port 5000:

```bash
sudo nft -f - <<'EOF'
flush ruleset

table bridge brf {
  chain forward {
    type filter hook forward priority 0; policy drop;

    ether type arp accept
    ct state established,related accept

    iifname "tap0" oifname "tap1" ip protocol tcp tcp dport 5000 accept
    iifname "tap1" oifname "tap0" ip protocol tcp tcp dport 5000 accept

    iifname "tap0" oifname "tap1" ip protocol icmp drop
    iifname "tap1" oifname "tap0" ip protocol icmp drop

    iifname "tap0" oifname "tap1" ip protocol udp drop
    iifname "tap1" oifname "tap0" ip protocol udp drop

    iifname "tap0" oifname "tap1" ip protocol tcp drop
    iifname "tap1" oifname "tap0" ip protocol tcp drop
  }
}
EOF
```

### 4.6 Set up bandwidth shaping

```bash
./framework_setup/scripts/network_speed.sh
```

This creates HTB tc classes: VM-to-host at 200 Mbit/s, VM-to-VM at 2 Mbit/s (initial). The `distance_bandwidth_shaper_node` dynamically updates the VM-to-VM rate based on inter-robot distance.

### 4.7 SSH into VMs

```bash
./framework_setup/scripts/robot1_ssh.sh
./framework_setup/scripts/robot2_ssh.sh
```

## Step 5: Running the System

### 5.1 Launch simulation (on host or a VM with Gazebo)

```bash
# Launch world
ros2 launch mrg_slam_sim marsyard2020.launch.py

# Launch robots
ros2 launch mrg_slam_sim dual_robot_sim.launch.py
```

### 5.2 Launch SLAM instances (one per VM)

On **robot1**:
```bash
ros2 launch mrg_slam mrg_slam.launch.py \
  model_namespace:=atlas x:=-15.0 y:=13.5 z:=1.1
```

On **robot2**:
```bash
ros2 launch mrg_slam mrg_slam.launch.py \
  model_namespace:=bestla x:=-15.0 y:=-13.0 z:=1.1
```

### 5.3 Launch bandwidth shaper and GUI (on host)

```bash
# Distance-based bandwidth shaper
ros2 run distance_bandwidth_shaper distance_bandwidth_shaper

# GUI dashboard
ros2 run distance_bandwidth_shaper mrg_slam_gui
```

### 5.4 (Optional) Teleoperation

```bash
# Control atlas
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r __node:=teleop_twist_keyboard_node_atlas \
  -r /cmd_vel:=/atlas/cmd_vel

# Control bestla
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r __node:=teleop_twist_keyboard_node_bestla \
  -r /cmd_vel:=/bestla/cmd_vel
```

### 5.5 Visualization

```bash
rviz2 -d Multi-Robot-Graph-SLAM/src/mrg_slam/rviz/mrg_slam.rviz \
  --ros-args -p use_sim_time:=true
```

## Verification

```bash
# Check link quality topic
ros2 topic echo /link_quality

# Check SLAM status
ros2 topic echo /atlas/mrg_slam/slam_status

# Verify bandwidth shaping
# On robot2:
iperf3 -s
# On robot1:
iperf3 -c 192.168.60.3
```
