# Adadptive Network-Aware Multi-Robot Graph SLAM

This project extends [Multi-Robot Graph SLAM](https://github.com/aserbremen/Multi-Robot-Graph-SLAM) with **distance-dependent bandwidth shaping** and **adaptive point cloud transmission**, enabling realistic simulation of multi-robot SLAM over constrained wireless links.

## Architecture

```
 ┌─────────────────────────────────────────────────────────────────┐
 │                        HOST MACHINE                             │
 │                                                                 │
 │  distance_bandwidth_shaper_node                                 │
 │    - reads /atlas/odom & /bestla/odom                           │
 │    - computes inter-robot distance                              │
 │    - applies tc rules on tap0/tap1 (HTB class 1:20)             │
 │    - publishes /link_quality (LinkQuality msg)                  │
 │                                                                 │
 │  mrg_slam_gui (PyQt5)                                           │
 │    - distance, bandwidth, latency gauges                        │
 │    - SLAM exchange stats, loop closure counts                   │
 │    - estimated transmit tier (EXCELLENT/GOOD/FAIR/POOR)         │
 │                                                                 │
 │         tap0 ──┐            ┌── tap1                            │
 │                └── br0 ─────┘                                   │
 │                  (shaped)                                       │
 └────────┬──────────────────────────────┬─────────────────────────┘
          │                              │
 ┌────────▼─────────────┐     ┌───────────▼────────────┐
 │   QEMU VM: robot1    │     │   QEMU VM: robot2      │
 │   (atlas)            │     │   (bestla)             │
 │                      │     │                        │
 │   mrg_slam node      │     │   mrg_slam node        │
 │     + adaptive_cloud │     │     + adaptive_cloud   │
 │       _transmitter   │     │       _transmitter     │
 │                      │     │                        │
 │   Subscribes to      │     │   Subscribes to        │
 │   /link_quality      │     │   /link_quality        │
 │   -> downsample      │     │   -> downsample        │
 │      point clouds    │     │      point clouds      │
 │      before TX       │     │      before TX         │
 └──────────────────────┘     └────────────────────────┘
```

## Components

### distance_bandwidth_shaper (Python ROS2 package)
Location: [`src/distance_bandwidth_shaper/`](src/distance_bandwidth_shaper/)

- **distance_bandwidth_shaper_node**: Computes inter-robot distance from odometry, applies Linux `tc` bandwidth shaping rules on the bridge interfaces, and publishes `LinkQuality` messages. Uses a log-distance path loss model to map distance to realistic wireless bandwidth.
- **mrg_slam_gui**: PyQt5 dashboard showing real-time distance, bandwidth, latency, SLAM graph exchange statistics, loop closure counts, and the estimated transmit tier.
- **robot_planned_path** (`robot_planned_path_updated.py`): Drives both robots along a deterministic waypoint path so the same trajectory can be replayed across experiments.

### adaptive_cloud_transmitter.hpp (C++ header)
Location: [`src/adaptive_cloud_transmitter.hpp`](src/adaptive_cloud_transmitter.hpp)

Integrates into the `mrg_slam` node to apply bandwidth-aware point cloud downsampling at transmission time. Subscribes to `/link_quality` and selects the minimum voxel grid downsampling needed so that the estimated transmit time fits within the time budget:

| Tier      | Leaf Size | Approx. Compression |
|-----------|-----------|---------------------|
| EXCELLENT | None      | 100% (original)     |
| GOOD      | 0.13 m    | ~21%                |
| FAIR      | 0.20 m    | ~12%                |
| POOR      | 1.20 m    | ~1%                 |

### Modified mrg_slam C++ components
Location: [`src/`](src/)

- **`mrg_slam_component.cpp`**: Instantiates the `AdaptiveCloudTransmitter` and uses it to downsample keyframe clouds before inter-robot graph exchange. Populates `SlamStatus` messages with graph exchange statistics (keyframes, edges, bytes transferred).
- **`prefiltering_component.cpp`**: Modified prefiltering (voxel/range filtering) to integrate with the bandwidth-aware pipeline.
- **`scan_matching_odometry_component.cpp`**: Modified scan matching odometry to align with the adaptive transmission policy.

### mrg_slam.yaml (Configuration)
Location: [`src/mrg_slam.yaml`](src/mrg_slam.yaml)

Updated config with the `adaptive_tx.*` parameters (`link_quality_topic`, `target_send_hz`, voxel leaf sizes for GOOD/FAIR/POOR tiers) plus tuning for loop closure thresholds.

### Custom / Modified Messages
Location: [`src/mrg_slam_msgs_changes/`](src/mrg_slam_msgs_changes/)

- **LinkQuality.msg** (new): `distance_m`, `bandwidth_mbit`, `burst_kbit`, `latency_ms`
- **AdaptiveTxStats.msg** (new): Per-transmission stats from the adaptive cloud transmitter (selected tier, bytes in/out, etc.)
- **SlamStatus.msg** (modified): Added `last_exchange_keyframes`, `last_exchange_edges`, `last_exchange_bytes`, `total_received_bytes`

### Multi-Robot-Graph-SLAM (git submodule)
The upstream framework by [aserbremen](https://github.com/aserbremen/Multi-Robot-Graph-SLAM), providing `mrg_slam`, `mrg_slam_msgs`, `mrg_slam_sim`, and related packages.

## Quick Start

```bash
git clone --recurse-submodules https://usergit.vcs.mmi.rwth-aachen.de/max.hermans/research-project-shubham.git
cd research-project-shubham
./framework_setup/scripts/setup_mrg_slam.sh
```

See [framework_setup/README.md](framework_setup/README.md) for detailed setup instructions including QEMU VM creation, network configuration, and running the full system.

## Acknowledgments

- Multi-Robot Graph SLAM framework: [aserbremen/Multi-Robot-Graph-SLAM](https://github.com/aserbremen/Multi-Robot-Graph-SLAM)
- Based on [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam) by koide3
