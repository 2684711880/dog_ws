#!/usr/bin/env python3
"""Realtime LF foot endpoint monitor from /joint_states.

This script subscribes to LF joint commands, reconstructs absolute leg angles
using stand offsets (computed by the same IK branch as locomotion_core_node),
then computes LF foot (x, y, z) with forward kinematics and plots it live.
"""

import argparse
import math
import sys
import threading
from collections import deque
from typing import Deque, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError as exc:
    print("matplotlib is required. Install it first, e.g. `pip install matplotlib`.", file=sys.stderr)
    raise exc


Sample = Tuple[float, float, float, float]  # (stamp_s, x, y, z)


class LFFootMonitor(Node):
    """Subscribe LF joints and reconstruct LF foot endpoint."""

    LF_JOINT_1 = "LF_Joint_1"
    LF_JOINT_2 = "LF_Joint_2"
    LF_JOINT_3 = "LF_Joint_3"

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("lf_foot_monitor")
        self.topic = args.topic
        self.history_sec = args.history_sec
        self.verbose = args.verbose

        # Geometry params from docs/重构.md (hip_offset_in_body + height_default).
        # LF hip offset in body frame: {+0.15, +0.05, 0.0}
        # Stand foot in hip frame = hip_offset sign-flipped projection:
        #   x: foot ahead of hip (~body_front - hip_x) -> +0.06
        #   y: foot left of hip   (~body_left  - hip_y) -> +0.0885
        # These values are kept from the original calibration; update if you
        # re-measure the actual robot.
        self.thigh_length = 0.21       # docs 重构.md L242
        self.shank_length = 0.21       # docs 重构.md L243
        self.hip_offset_x = 0.15       # docs 重构.md L250 (LF)
        self.hip_offset_y = 0.05        # docs 重构.md L250 (LF)
        self.height_default = 0.29      # docs 重构.md L113
        self.knee_gear_ratio = 2.0      # docs 重构.md L244

        # Stand foot position in hip frame (calibrated values, keep until re-measured)
        self.stand_foot_x = 0.06
        self.stand_foot_y = 0.0885

        self.t1_0, self.t2_0, self.t3_0 = self._foot_ik(
            self.stand_foot_x, self.stand_foot_y, -self.height_default
        )

        self._samples: Deque[Sample] = deque()
        self._lock = threading.Lock()
        self._last_print_stamp = -1.0
        self._missing_joint_warned = False

        self.sub = self.create_subscription(
            JointState, self.topic, self._joint_cb, 20
        )

        self.get_logger().info(
            f"LF monitor started. topic={self.topic}, "
            f"stand_offsets(rad)=({self.t1_0:.4f}, {self.t2_0:.4f}, {self.t3_0:.4f})"
        )

    def _foot_ik(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """Same IK branch as locomotion_planner::foot_ik for consistency.
        
        Note: z is negative (downward) in the robot frame.
        """
        t1 = math.atan2(y, x)

        l_horizontal = math.sqrt(x * x + y * y)
        distance = math.sqrt(l_horizontal * l_horizontal + z * z)

        max_reach = self.thigh_length + self.shank_length
        min_reach = abs(self.thigh_length - self.shank_length)
        if distance > max_reach or distance < min_reach:
            raise ValueError(
                f"Stand target unreachable: ({x:.3f}, {y:.3f}, {z:.3f}), "
                f"distance={distance:.3f}, range=[{min_reach:.3f}, {max_reach:.3f}]"
            )

        cos_t3 = (
            (distance * distance - self.thigh_length * self.thigh_length - self.shank_length * self.shank_length)
            / (2.0 * self.thigh_length * self.shank_length)
        )
        cos_t3 = max(-1.0, min(1.0, cos_t3))
        t3_leg = math.acos(cos_t3)

        alpha = math.atan2(z, l_horizontal)
        tmp = self.shank_length * math.sin(t3_leg) / max(distance, 1e-6)
        tmp = max(-1.0, min(1.0, tmp))
        beta = math.asin(tmp)
        t2 = alpha - beta
        return t1, t2, t3_leg

    def _restore_angles_and_fk(self, j1: float, j2: float, j3: float) -> Tuple[float, float, float]:
        # Joint command encoding (from locomotion_planner.cpp + 重构.md L244):
        #   j1_cmd = (t1 - t1_0)  (with J1 decoupling gain)
        #   j2_cmd = t2 - t2_0
        #   j3_cmd = (t3_leg - t3_0) * knee_gear_ratio
        # So to recover absolute angles (knee_gear_ratio = 2.0):
        t1 = self.t1_0 + j1
        t2 = self.t2_0 + j2
        t3 = self.t3_0 + j3 / self.knee_gear_ratio

        # Forward kinematics (t3 = knee angle, t2 = hip pitch)
        r = self.thigh_length * math.cos(t2) + self.shank_length * math.cos(t2 + t3)
        z_up = -(self.thigh_length * math.sin(t2) + self.shank_length * math.sin(t2 + t3))
        x = r * math.cos(t1)
        y = r * math.sin(t1)
        return x, y, z_up

    def _joint_cb(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position):
            self.get_logger().warn("Skip malformed JointState: name/position length mismatch")
            return

        idx_map = {name: i for i, name in enumerate(msg.name)}
        required = {self.LF_JOINT_1, self.LF_JOINT_2, self.LF_JOINT_3}
        if not required.issubset(set(idx_map.keys())):
            if not self._missing_joint_warned:
                self.get_logger().warn(
                    f"Waiting LF joints in {self.topic}. Need: "
                    f"{self.LF_JOINT_1}, {self.LF_JOINT_2}, {self.LF_JOINT_3}"
                )
                self._missing_joint_warned = True
            return
        self._missing_joint_warned = False

        j1 = float(msg.position[idx_map[self.LF_JOINT_1]])
        j2 = float(msg.position[idx_map[self.LF_JOINT_2]])
        j3 = float(msg.position[idx_map[self.LF_JOINT_3]])
        x, y, z = self._restore_angles_and_fk(j1, j2, j3)

        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            stamp_s = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
        else:
            stamp_s = self.get_clock().now().nanoseconds * 1e-9

        with self._lock:
            self._samples.append((stamp_s, x, y, z))
            while self._samples and (stamp_s - self._samples[0][0]) > self.history_sec:
                self._samples.popleft()

        if self.verbose and (self._last_print_stamp < 0.0 or (stamp_s - self._last_print_stamp) >= 0.1):
            self._last_print_stamp = stamp_s
            print(f"LF foot xyz: x={x:+.4f} m, y={y:+.4f} m, z={z:+.4f} m")

    def get_snapshot(self) -> Optional[Dict[str, object]]:
        with self._lock:
            if not self._samples:
                return None
            samples = list(self._samples)

        t0 = samples[0][0]
        ts = [s[0] - t0 for s in samples]
        xs = [s[1] for s in samples]
        ys = [s[2] for s in samples]
        zs = [s[3] for s in samples]

        return {
            "t": ts,
            "x": xs,
            "y": ys,
            "z": zs,
            "count": len(samples),
            "duration": ts[-1] if ts else 0.0,
            "x_range": max(xs) - min(xs),
            "y_range": max(ys) - min(ys),
            "z_range": max(zs) - min(zs),
            "latest": (xs[-1], ys[-1], zs[-1]),
        }


def _set_axis_limits(ax: plt.Axes, xs: list, ys: list, min_span: float = 0.02) -> None:
    if not xs or not ys:
        return

    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)

    x_span = max(x_max - x_min, min_span)
    y_span = max(y_max - y_min, min_span)

    x_mid = 0.5 * (x_max + x_min)
    y_mid = 0.5 * (y_max + y_min)

    x_margin = 0.1 * x_span + 1e-6
    y_margin = 0.1 * y_span + 1e-6

    ax.set_xlim(x_mid - 0.5 * x_span - x_margin, x_mid + 0.5 * x_span + x_margin)
    ax.set_ylim(y_mid - 0.5 * y_span - y_margin, y_mid + 0.5 * y_span + y_margin)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Subscribe /joint_states and monitor LF foot endpoint trajectory in real time."
    )
    parser.add_argument("--topic", default="/joint_states", help="JointState topic to subscribe.")
    parser.add_argument(
        "--history-sec",
        type=float,
        default=8.0,
        help="History window seconds for plotting and range stats.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Plot refresh rate in Hz.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print realtime LF foot xyz to terminal.",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.history_sec <= 0.0:
        parser.error("--history-sec must be > 0")
    if args.rate <= 0.0:
        parser.error("--rate must be > 0")

    rclpy.init(args=None)
    node = LFFootMonitor(args)
    stop_event = threading.Event()

    def spin_worker() -> None:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)

    spin_thread = threading.Thread(target=spin_worker, daemon=True)
    spin_thread.start()

    fig, (ax_xz, ax_yz, ax_t) = plt.subplots(3, 1, figsize=(10, 11))
    fig.suptitle("LF Foot Monitor from /joint_states", fontsize=14)

    ax_xz.set_title("X-Z Trajectory (forward / vertical)")
    ax_xz.set_xlabel("X (m, forward +)")
    ax_xz.set_ylabel("Z (m, up +)")
    ax_xz.grid(True, alpha=0.3)
    ax_xz.set_aspect("equal", adjustable="box")

    ax_yz.set_title("Y-Z Trajectory (left / vertical)")
    ax_yz.set_xlabel("Y (m, left +)")
    ax_yz.set_ylabel("Z (m, up +)")
    ax_yz.grid(True, alpha=0.3)
    ax_yz.set_aspect("equal", adjustable="box")

    ax_t.set_title("Time Series in Current Window")
    ax_t.set_xlabel("Time in Window (s)")
    ax_t.set_ylabel("Position (m)")
    ax_t.grid(True, alpha=0.3)

    line_xz, = ax_xz.plot([], [], color="tab:blue", linewidth=1.5, label="LF path")
    dot_xz, = ax_xz.plot([], [], marker="o", color="tab:red", markersize=5)
    line_yz, = ax_yz.plot([], [], color="tab:green", linewidth=1.5, label="LF path")
    dot_yz, = ax_yz.plot([], [], marker="o", color="tab:red", markersize=5)

    line_xt, = ax_t.plot([], [], color="tab:blue", linewidth=1.2, label="x(t)")
    line_yt, = ax_t.plot([], [], color="tab:green", linewidth=1.2, label="y(t)")
    line_zt, = ax_t.plot([], [], color="tab:orange", linewidth=1.2, label="z(t)")
    ax_t.legend(loc="upper left")

    status_text = ax_t.text(
        0.01, 0.98, "Waiting for LF joints...",
        transform=ax_t.transAxes, va="top", ha="left", fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.7, "edgecolor": "none"},
    )
    stats_text = ax_t.text(
        0.99, 0.98, "",
        transform=ax_t.transAxes, va="top", ha="right", fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.7, "edgecolor": "none"},
    )

    def _update(_frame_idx: int):
        snap = node.get_snapshot()
        if snap is None:
            status_text.set_text(f"Waiting for LF joints on {args.topic} ...")
            stats_text.set_text("")
            return (
                line_xz, dot_xz, line_yz, dot_yz, line_xt, line_yt, line_zt,
                status_text, stats_text,
            )

        ts = snap["t"]
        xs = snap["x"]
        ys = snap["y"]
        zs = snap["z"]

        line_xz.set_data(xs, zs)
        dot_xz.set_data([xs[-1]], [zs[-1]])
        line_yz.set_data(ys, zs)
        dot_yz.set_data([ys[-1]], [zs[-1]])

        line_xt.set_data(ts, xs)
        line_yt.set_data(ts, ys)
        line_zt.set_data(ts, zs)

        _set_axis_limits(ax_xz, xs, zs, min_span=0.01)
        _set_axis_limits(ax_yz, ys, zs, min_span=0.01)
        _set_axis_limits(ax_t, ts, xs + ys + zs, min_span=0.02)

        latest_x, latest_y, latest_z = snap["latest"]
        status_text.set_text(
            f"topic: {args.topic}\n"
            f"latest xyz: ({latest_x:+.4f}, {latest_y:+.4f}, {latest_z:+.4f}) m\n"
            f"window: {snap['duration']:.2f}s, samples: {snap['count']}"
        )
        stats_text.set_text(
            f"range in window\n"
            f"dx = {1000.0 * snap['x_range']:.2f} mm\n"
            f"dy = {1000.0 * snap['y_range']:.2f} mm\n"
            f"dz = {1000.0 * snap['z_range']:.2f} mm"
        )

        return (
            line_xz, dot_xz, line_yz, dot_yz, line_xt, line_yt, line_zt,
            status_text, stats_text,
        )

    refresh_ms = max(20, int(round(1000.0 / args.rate)))
    anim = FuncAnimation(fig, _update, interval=refresh_ms, blit=False)
    _ = anim  # Keep reference.

    def _on_close(_event) -> None:
        stop_event.set()

    fig.canvas.mpl_connect("close_event", _on_close)
    plt.tight_layout(rect=[0.0, 0.0, 1.0, 0.97])

    exit_code = 0
    try:
        plt.show()
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        stop_event.set()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
