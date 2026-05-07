#!/usr/bin/env python3
import math
import subprocess
#from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from mrg_slam_msgs.msg import LinkQuality

#@dataclass
#class LinkPair:
    #tap: str
    #ifb: str

def bandwidth_from_distance(
    d_m: float,
    bw_max_mbps: float = 180.0,
    bw_min_mbps: float = 2.0,
    d_ref_m: float = 1.0,
    path_loss_exp: float = 2.8,
    d_transition: float = 5.0,
    sharpness: float = 3.0,
) -> float:
    # log-distance path loss with logistic falloff around d_transition.
    # path_loss_exp ~2.0 free space, ~2.2-3.0 cluttered indoor/outdoor.
    d = max(d_m, 0.05)

    path_loss_db = 10.0 * path_loss_exp * math.log10(1.0 + d / d_ref_m)

    transition_loss = 10 * path_loss_exp * math.log10(d_transition / d_ref_m + 1)

    quality = 1 / (1 + math.exp((path_loss_db - transition_loss) / sharpness))

    bw = bw_min_mbps + (bw_max_mbps - bw_min_mbps) * quality

    return max(bw_min_mbps, min(bw, bw_max_mbps))


def latency_from_distance(
    d_m: float,
    lat_min_ms: float = 1.0,
    lat_max_ms: float = 150.0,
    d_ref_m: float = 1.0,
    path_loss_exp: float = 2.8,
    d_transition: float = 5.0,
    sharpness: float = 3.0,
) -> float:
    # same path-loss/logistic shape as bandwidth_from_distance, inverted
    # (quality=1 -> low latency, quality=0 -> high latency)
    d = max(d_m, 0.05)
    path_loss_db = 10.0 * path_loss_exp * math.log10(1.0 + d / d_ref_m)
    transition_loss = 10.0 * path_loss_exp * math.log10(d_transition / d_ref_m + 1)
    quality = 1.0 / (1.0 + math.exp((path_loss_db - transition_loss) / sharpness))
    latency = lat_max_ms - (lat_max_ms - lat_min_ms) * quality
    return max(lat_min_ms, min(latency, lat_max_ms))


class DistanceBandwidthFromOdomGT(Node):
    def __init__(self):
        super().__init__("distance_bw_from_odom_ground_truth")

        self.declare_parameter("robot1_odom_topic", "/atlas/odom_ground_truth")
        self.declare_parameter("robot2_odom_topic", "/bestla/odom_ground_truth")

        # rate control
        self.declare_parameter("hold_time_s", 5.0)
        self.declare_parameter("min_rate_change_mbit", 0.1)

        
        self.topic1 = self.get_parameter("robot1_odom_topic").value
        self.topic2 = self.get_parameter("robot2_odom_topic").value


        self.hold_time_s = float(self.get_parameter("hold_time_s").value)
        self.min_rate_change = float(self.get_parameter("min_rate_change_mbit").value)


        self.p1: Optional[Tuple[float, float, float]] = None
        self.p2: Optional[Tuple[float, float, float]] = None

        self.last_rate: Optional[float] = None
        self.last_latency: Optional[float] = None
        self.last_change_time = self.get_clock().now()

        self.sub1 = self.create_subscription(Odometry, self.topic1, self._cb1, 10)
        self.sub2 = self.create_subscription(Odometry, self.topic2, self._cb2, 10)
        self.lq_pub = self.create_publisher(LinkQuality, "/link_quality", 10)
        self.dist_pub = self.create_publisher(String, "/distance", 10)

        self.get_logger().info(f"Subscribing robot1 odom: {self.topic1}")
        self.get_logger().info(f"Subscribing robot2 odom: {self.topic2}")

        self.timer = self.create_timer(1.0, self.update)  # 1 Hz

    def _cb1(self, msg: Odometry):
        p = msg.pose.pose.position
        self.p1 = (p.x, p.y, p.z)

    def _cb2(self, msg: Odometry):
        p = msg.pose.pose.position
        self.p2 = (p.x, p.y, p.z)

    def _distance(self, a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return round(math.sqrt(dx * dx + dy * dy + dz * dz), 3)

    def _bandwidth_from_distance(self, d):
        return bandwidth_from_distance(d)

    def _latency_from_distance(self, d):
        return latency_from_distance(d)

    def update(self):
        if self.p1 is None or self.p2 is None:
            return

        d = self._distance(self.p1, self.p2)
        target_rate = self._bandwidth_from_distance(d)
        target_latency = self._latency_from_distance(d)

        lq = LinkQuality()
        lq.distance_m = float(d)
        lq.bandwidth_mbit = float(target_rate)
        lq.burst_kbit = 64.0
        lq.latency_ms = float(target_latency)
        self.lq_pub.publish(lq)

        distance = String()
        distance.data = f"Distance between atlas and bestla:={d}"
        self.dist_pub.publish(distance)

        now = self.get_clock().now()

        if self.last_rate is not None:
            rate_changed = abs(target_rate - self.last_rate) >= self.min_rate_change
            latency_changed = (self.last_latency is None or
                               abs(target_latency - self.last_latency) >= 10.0)
            if not rate_changed and not latency_changed:
                return
            if (now - self.last_change_time).nanoseconds < int(self.hold_time_s * 1e9):
                return

        self.set_vm_vm_rate(target_rate, target_latency)
        self.last_rate = target_rate
        self.last_latency = target_latency
        self.last_change_time = now

    def set_vm_vm_rate(self, target_rate: float, target_latency: float) -> None:
        rate_str = f"{target_rate}mbit"
        delay_str = f"{target_latency}ms"

        for dev in ["tap0", "tap1"]:
            self._run([
                "sudo", "tc", "class", "change",
                "dev", dev,
                "parent", "1:",
                "classid", "1:20",
                "htb",
                "rate", rate_str,
                "ceil", rate_str
            ])
            self._run([
                "sudo", "tc", "qdisc", "replace",
                "dev", dev,
                "parent", "1:20",
                "handle", "20:",
                "netem",
                "delay", delay_str,
                "limit", "10000",
            ])
        self.get_logger().info(
            f"atlas <-> bestla: rate={rate_str}, delay={delay_str}"
        )


    def _run(self, cmd: List[str]):
        subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def main():
    rclpy.init()
    node = DistanceBandwidthFromOdomGT()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
