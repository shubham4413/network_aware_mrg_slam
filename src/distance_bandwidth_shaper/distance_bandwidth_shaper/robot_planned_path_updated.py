#!/usr/bin/env python3
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


Waypoint = Tuple[float, float]


ATLAS_WAYPOINTS: List[Waypoint] = [
    (-10.5, 12.8),
    (12.0, 12.8),
    (12.0, -12.2),
    (-12.0, -12.2),
    (-12.0, 12.2),
    #(-10.8, 12.8),
]

BESTLA_WAYPOINTS: List[Waypoint] = [
    (-9.5, 11.2),
    (1.5, 11.0),
    (1.5, -11.0),
    (-12.0, -11.0),
    (-12.0, 10.8),
    (-10.2, 11.2),
]



def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class RobotFollower:
    name: str
    cmd_topic: str
    odom_topic: str
    waypoints: List[Waypoint]
    start_delay: float = 0.0
    waypoint_index: int = 0
    x: Optional[float] = None
    y: Optional[float] = None
    yaw: Optional[float] = None
    finished: bool = False
    publisher: Optional[object] = field(default=None, repr=False)

    def has_pose(self) -> bool:
        return self.x is not None and self.y is not None and self.yaw is not None

    def current_waypoint(self) -> Optional[Waypoint]:
        if self.finished or self.waypoint_index >= len(self.waypoints):
            return None
        return self.waypoints[self.waypoint_index]


class PlannedPathFollower(Node):
    def __init__(self) -> None:
        super().__init__("planned_path_follower")

        atlas_odom_default = "/atlas/odom_ground_truth"
        bestla_odom_default = "/bestla/odom_ground_truth"
        self.declare_parameter("atlas_odom_topic", atlas_odom_default)
        self.declare_parameter("bestla_odom_topic", bestla_odom_default)

        # motion tuning, kept conservative
        self.declare_parameter("max_linear_speed", 0.28)
        self.declare_parameter("min_linear_speed", 0.05)
        self.declare_parameter("max_angular_speed", 0.35)
        self.declare_parameter("k_linear", 0.45)
        self.declare_parameter("k_angular", 1.25)
        self.declare_parameter("goal_tolerance", 0.30)
        self.declare_parameter("turn_in_place_angle", 0.35)  # rad
        self.declare_parameter("slowdown_radius", 1.8)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("atlas_start_delay", 0.0)
        self.declare_parameter("bestla_start_delay", 4.0)

        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.min_linear_speed = float(self.get_parameter("min_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.k_linear = float(self.get_parameter("k_linear").value)
        self.k_angular = float(self.get_parameter("k_angular").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.turn_in_place_angle = float(self.get_parameter("turn_in_place_angle").value)
        self.slowdown_radius = float(self.get_parameter("slowdown_radius").value)
        control_rate_hz = float(self.get_parameter("control_rate_hz").value)

        self.robots = {
            "atlas": RobotFollower(
                name="atlas",
                cmd_topic="/atlas/cmd_vel",
                odom_topic=str(self.get_parameter("atlas_odom_topic").value),
                waypoints=ATLAS_WAYPOINTS,
                start_delay=float(self.get_parameter("atlas_start_delay").value),
            ),
            "bestla": RobotFollower(
                name="bestla",
                cmd_topic="/bestla/cmd_vel",
                odom_topic=str(self.get_parameter("bestla_odom_topic").value),
                waypoints=BESTLA_WAYPOINTS,
                start_delay=float(self.get_parameter("bestla_start_delay").value),
            ),
        }

        for robot in self.robots.values():
            robot.publisher = self.create_publisher(Twist, robot.cmd_topic, 10)
            self.create_subscription(
                Odometry,
                robot.odom_topic,
                lambda msg, robot_name=robot.name: self.odom_callback(robot_name, msg),
                10,
            )

        self.start_time = self.get_clock().now()
        self.last_status_log_time = -999.0
        self.timer = self.create_timer(1.0 / control_rate_hz, self.control_loop)

        self.get_logger().info("PlannedPathFollower started")
        for robot in self.robots.values():
            self.get_logger().info(
                f"{robot.name}: odom={robot.odom_topic}, cmd={robot.cmd_topic}, "
                f"start_delay={robot.start_delay:.1f}s, waypoints={len(robot.waypoints)}"
            )

    def odom_callback(self, robot_name: str, msg: Odometry) -> None:
        robot = self.robots[robot_name]
        pose = msg.pose.pose
        robot.x = float(pose.position.x)
        robot.y = float(pose.position.y)
        robot.yaw = yaw_from_quaternion(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )

    def make_stop_cmd(self) -> Twist:
        return Twist()

    def compute_cmd(self, robot: RobotFollower) -> Twist:
        cmd = Twist()
        waypoint = robot.current_waypoint()
        if waypoint is None or not robot.has_pose():
            return cmd

        wx, wy = waypoint
        dx = wx - robot.x
        dy = wy - robot.y
        distance = math.hypot(dx, dy)

        if distance < self.goal_tolerance:
            robot.waypoint_index += 1
            if robot.waypoint_index >= len(robot.waypoints):
                robot.finished = True
                self.get_logger().info(f"{robot.name} finished trajectory")
                return cmd
            next_wp = robot.waypoints[robot.waypoint_index]
            self.get_logger().info(
                f"{robot.name} reached waypoint {robot.waypoint_index}/{len(robot.waypoints)}; "
                f"next={next_wp}"
            )
            return cmd

        desired_yaw = math.atan2(dy, dx)
        yaw_error = wrap_to_pi(desired_yaw - robot.yaw)

        if abs(yaw_error) > self.turn_in_place_angle:
            cmd.linear.x = 0.0
            cmd.angular.z = clamp(self.k_angular * yaw_error, -self.max_angular_speed, self.max_angular_speed)
            return cmd
        linear_cap = self.max_linear_speed
        if distance < self.slowdown_radius:
            linear_cap = max(self.min_linear_speed, self.max_linear_speed * (distance / self.slowdown_radius))

        heading_scale = max(0.25, math.cos(abs(yaw_error)))
        linear_cmd = clamp(self.k_linear * distance, self.min_linear_speed, linear_cap)
        cmd.linear.x = linear_cmd * heading_scale
        cmd.angular.z = clamp(self.k_angular * yaw_error, -self.max_angular_speed, self.max_angular_speed)
        return cmd

    def control_loop(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        for robot in self.robots.values():
            if elapsed < robot.start_delay:
                robot.publisher.publish(self.make_stop_cmd())
                continue

            if robot.finished or not robot.has_pose():
                robot.publisher.publish(self.make_stop_cmd())
                continue

            robot.publisher.publish(self.compute_cmd(robot))

        if elapsed - self.last_status_log_time > 5.0:
            self.last_status_log_time = elapsed
            for robot in self.robots.values():
                wp = robot.current_waypoint()
                pose_str = "no odom yet"
                if robot.has_pose():
                    pose_str = f"pose=({robot.x:.2f}, {robot.y:.2f}, yaw={robot.yaw:.2f})"
                self.get_logger().info(
                    f"t={elapsed:.1f}s | {robot.name}: wp_idx={robot.waypoint_index}, wp={wp}, {pose_str}"
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannedPathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for robot in node.robots.values():
            if robot.publisher is not None:
                robot.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()