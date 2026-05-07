import rclpy
from rclpy.node import Node
from mrg_slam_msgs.msg import SlamStatus, LinkQuality, AdaptiveTxStats
from visualization_msgs.msg import MarkerArray
import sys
import time
import threading
import os
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QHBoxLayout
from PyQt5 import QtCore
from ament_index_python.packages import get_package_share_directory

# Loop closure edge colors from markers_publisher.cpp
PURPLE_R = 148.0 / 255.0
PINK_R = 227.0 / 255.0


def _load_target_send_hz():
    try:
        config_path = os.path.join(get_package_share_directory("mrg_slam"), "config", "mrg_slam.yaml")
        with open(config_path, "r") as f:
            for line in f:
                if "adaptive_tx.target_send_hz" in line and ":" in line:
                    no_comment = line.split("#")[0]
                    value_str = no_comment.split(":", 1)[1].strip()
                    return float(value_str)
    except Exception:
        pass
    return 4.0


TARGET_SEND_HZ = _load_target_send_hz()
print(f"[mrg_slam_gui] Loaded TARGET_SEND_HZ = {TARGET_SEND_HZ}")


def count_loop_edges(marker_array):
    local_lc = 0
    inter_lc = 0
    for marker in marker_array.markers:
        if marker.ns != "main_edges":
            continue
        for i in range(0, len(marker.colors), 2):
            r = marker.colors[i].r
            if abs(r - PURPLE_R) < 0.01:
                local_lc += 1
            elif abs(r - PINK_R) < 0.01:
                inter_lc += 1
    return local_lc, inter_lc


class MrgSlamGUI(Node):

    def __init__(self):
        super().__init__("gui_for_bandwidth_aware_mrg_slam")

        self.lq_sub = self.create_subscription(LinkQuality, "/link_quality", self.link_quality_cb, 10)
        self.atlas_stats_sub = self.create_subscription(
            SlamStatus, "/atlas/mrg_slam/slam_status", self.atlas_stats_cb, 10)
        self.bestla_stats_sub = self.create_subscription(
            SlamStatus, "/bestla/mrg_slam/slam_status", self.bestla_stats_cb, 10)
        self.atlas_markers_sub = self.create_subscription(
            MarkerArray, "/atlas/mrg_slam/markers", self.atlas_markers_cb, 1)
        self.bestla_markers_sub = self.create_subscription(
            MarkerArray, "/bestla/mrg_slam/markers", self.bestla_markers_cb, 1)

        self.atlas_tx_stats_sub = self.create_subscription(
            AdaptiveTxStats, "/atlas/adaptive_tx/stats", self.tx_stats_cb, 10)
        self.bestla_tx_stats_sub = self.create_subscription(
            AdaptiveTxStats, "/bestla/adaptive_tx/stats", self.tx_stats_cb, 10)

        self.dist = 0.0
        self.band = 0.0
        self.latency = 0.0
        self.atlas_stats = {"last_kf": 0, "last_edges": 0, "last_bytes": 0, "total_bytes": 0, "exchanging": False}
        self.bestla_stats = {"last_kf": 0, "last_edges": 0, "last_bytes": 0, "total_bytes": 0, "exchanging": False}

        self.tx_display = {
            "atlas":  {"tier": "---", "orig": 0, "sent": 0, "total_orig": 0, "total_sent": 0},
            "bestla": {"tier": "---", "orig": 0, "sent": 0, "total_orig": 0, "total_sent": 0},
        }

        self.atlas_local_lc = 0
        self.atlas_inter_lc = 0
        self.bestla_local_lc = 0
        self.bestla_inter_lc = 0

        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("Bandwidth Aware MrgSLAM GUI")

        self.outerlayout = QVBoxLayout()
        self.row1 = QHBoxLayout()
        self.row2a = QHBoxLayout()
        self.row2b = QHBoxLayout()
        self.row3 = QHBoxLayout()
        self.row4 = QHBoxLayout()
        self.row5 = QHBoxLayout()

        self.outerlayout.addLayout(self.row1)
        self.outerlayout.addLayout(self.row2a)
        self.outerlayout.addLayout(self.row2b)
        self.outerlayout.addLayout(self.row3)
        self.outerlayout.addLayout(self.row4)
        self.outerlayout.addLayout(self.row5)

        base_style = "QLabel {border: 1px solid black; font-size: 18pt; background-color: #c4c4c4;}"
        lc_style = "QLabel {border: 1px solid black; font-size: 18pt; background-color: #d5f5e3;}"
        tx_style = "QLabel {border: 1px solid black; font-size: 18pt; background-color: #fdebd0;}"

        self.label_bw = QLabel("Bandwidth: 0.0 Mbps")
        self.label_bw.setStyleSheet(base_style)
        self.label_latency = QLabel("Latency: 0.0 ms")
        self.label_latency.setStyleSheet(base_style)
        self.label_dist = QLabel("Distance: 0.0 m")
        self.label_dist.setStyleSheet(base_style)
        self.label_target_hz = QLabel(f"{TARGET_SEND_HZ} Hz    {1000/TARGET_SEND_HZ:.0f} ms")
        self.label_target_hz.setStyleSheet(base_style)
        self.row1.addWidget(self.label_bw)
        self.row1.addWidget(self.label_latency)
        self.row1.addWidget(self.label_dist)
        self.row1.addWidget(self.label_target_hz)

        self.label_atlas_tx = QLabel("Atlas TX: --- | Original: - | Sent: - | Reduction: 0%")
        self.label_atlas_tx.setStyleSheet(tx_style)
        self.row2a.addWidget(self.label_atlas_tx)

        self.label_bestla_tx = QLabel("Bestla TX: --- | Original: - | Sent: - | Reduction: 0%")
        self.label_bestla_tx.setStyleSheet(tx_style)
        self.row2b.addWidget(self.label_bestla_tx)

        self.label_atlas_stats = QLabel("Atlas: Last exchange: 0 KF, 0 edges, 0 B | Total received: 0 B")
        self.label_atlas_stats.setStyleSheet(base_style)
        self.row3.addWidget(self.label_atlas_stats)

        self.label_bestla_stats = QLabel("Bestla: Last exchange: 0 KF, 0 edges, 0 B | Total received: 0 B")
        self.label_bestla_stats.setStyleSheet(base_style)
        self.row4.addWidget(self.label_bestla_stats)

        self.label_atlas_lc = QLabel("Atlas LC: local 0 | inter 0")
        self.label_atlas_lc.setStyleSheet(lc_style)
        self.label_bestla_lc = QLabel("Bestla LC: local 0 | inter 0")
        self.label_bestla_lc.setStyleSheet(lc_style)
        self.row5.addWidget(self.label_atlas_lc)
        self.row5.addWidget(self.label_bestla_lc)

        self.window.setLayout(self.outerlayout)
        self.window.show()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.ui_update)  # type: ignore
        self.timer.start(200)

    def _format_bytes(self, b):
        if b < 1024:
            return f"{b} B"
        elif b < 1024 * 1024:
            return f"{b / 1024:.1f} KB"
        else:
            return f"{b / (1024 * 1024):.2f} MB"

    TIER_COLORS = {
        "FULL": "#2ecc71",
        "GOOD (VG 0.13m)": "#3498db",
        "FAIR (VG 0.20m)": "#f1c40f",
        "POOR (AVG 1.2m)": "#e74c3c",
    }

    def link_quality_cb(self, msg: LinkQuality):
        self.dist = msg.distance_m
        self.band = msg.bandwidth_mbit
        self.latency = msg.latency_ms

    def tx_stats_cb(self, msg: AdaptiveTxStats):
        robot = msg.robot_name
        if robot not in self.tx_display:
            return
        self.tx_display[robot]["tier"] = msg.tier_name
        self.tx_display[robot]["orig"] = msg.original_bytes
        self.tx_display[robot]["sent"] = min(msg.sent_bytes, msg.original_bytes)
        self.tx_display[robot]["total_orig"] = msg.total_original_bytes
        self.tx_display[robot]["total_sent"] = msg.total_sent_bytes

    def atlas_markers_cb(self, msg: MarkerArray):
        self._atlas_marker_count = getattr(self, '_atlas_marker_count', 0) + 1
        if self._atlas_marker_count % 5 == 0:
            self.atlas_local_lc, self.atlas_inter_lc = count_loop_edges(msg)

    def bestla_markers_cb(self, msg: MarkerArray):
        self._bestla_marker_count = getattr(self, '_bestla_marker_count', 0) + 1
        if self._bestla_marker_count % 5 == 0:
            self.bestla_local_lc, self.bestla_inter_lc = count_loop_edges(msg)

    def _update_stats(self, stats, msg):
        stats["exchanging"] = msg.in_graph_exchange
        if not msg.in_graph_exchange and msg.last_exchange_keyframes > 0:
            stats["last_kf"] = msg.last_exchange_keyframes
            stats["last_edges"] = msg.last_exchange_edges
            stats["last_bytes"] = msg.last_exchange_bytes
            stats["total_bytes"] = msg.total_received_bytes

    def atlas_stats_cb(self, msg: SlamStatus):
        self._update_stats(self.atlas_stats, msg)

    def bestla_stats_cb(self, msg: SlamStatus):
        self._update_stats(self.bestla_stats, msg)

    def _format_tx_row(self, name, data):
        tier = data["tier"]
        orig = data["orig"]
        sent = data["sent"]
        total_orig = data["total_orig"]
        total_sent = data["total_sent"]
        saved = total_orig - total_sent
        if orig > 0:
            reduction = max(0.0, (1.0 - sent / orig) * 100)
        else:
            reduction = 0.0
        return (f"{name} TX: {tier} | Original: {self._format_bytes(orig)} | "
                f"Sent: {self._format_bytes(sent)} | Reduction: {reduction:.0f}% | "
                f"Saved: {self._format_bytes(saved)}")

    def _tier_color(self, tier_name):
        return self.TIER_COLORS.get(tier_name, "#fdebd0")

    def ui_update(self):
        self.label_bw.setText(f"Bandwidth: {self.band:.1f} Mbps")
        self.label_latency.setText(f"Latency: {self.latency:.1f} ms")
        self.label_dist.setText(f"Distance: {self.dist:.1f} m")

        atlas_data = self.tx_display["atlas"]
        bestla_data = self.tx_display["bestla"]
        self.label_atlas_tx.setText(self._format_tx_row("Atlas", atlas_data))
        self.label_atlas_tx.setStyleSheet(
            f"QLabel {{border: 1px solid black; font-size: 18pt; background-color: {self._tier_color(atlas_data['tier'])};}}")
        self.label_bestla_tx.setText(self._format_tx_row("Bestla", bestla_data))
        self.label_bestla_tx.setStyleSheet(
            f"QLabel {{border: 1px solid black; font-size: 18pt; background-color: {self._tier_color(bestla_data['tier'])};}}")

        for stats, label, name in [
            (self.atlas_stats, self.label_atlas_stats, "Atlas"),
            (self.bestla_stats, self.label_bestla_stats, "Bestla"),
        ]:
            status = " [EXCHANGING]" if stats["exchanging"] else ""
            label.setText(
                f"{name}: Last exchange: {stats['last_kf']} KF, "
                f"{stats['last_edges']} edges, {self._format_bytes(stats['last_bytes'])} | "
                f"Total received: {self._format_bytes(stats['total_bytes'])}{status}"
            )

        self.label_atlas_lc.setText(
            f"Atlas LC: local {self.atlas_local_lc} | inter {self.atlas_inter_lc}")
        self.label_bestla_lc.setText(
            f"Bestla LC: local {self.bestla_local_lc} | inter {self.bestla_inter_lc}")

        for label, inter_count in [
            (self.label_atlas_lc, self.atlas_inter_lc),
            (self.label_bestla_lc, self.bestla_inter_lc),
        ]:
            if inter_count > 0:
                label.setStyleSheet(
                    "QLabel {border: 1px solid black; font-size: 18pt; background-color: #aed6f1;}")
            else:
                label.setStyleSheet(
                    "QLabel {border: 1px solid black; font-size: 18pt; background-color: #d5f5e3;}")


def spin_ros(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = MrgSlamGUI()

    ros_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    ros_thread.start()

    try:
        sys.exit(node.app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
