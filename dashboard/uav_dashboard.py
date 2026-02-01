#!/usr/bin/env python3
import argparse
import math
import threading
import time
from collections import deque
from typing import Deque, Optional, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Imu, MagneticField

import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


class DataSeries:
    def __init__(self, maxlen: int = 600) -> None:
        self.t: Deque[float] = deque(maxlen=maxlen)
        self.x: Deque[float] = deque(maxlen=maxlen)
        self.y: Deque[float] = deque(maxlen=maxlen)
        self.z: Deque[float] = deque(maxlen=maxlen)

    def append(self, t: float, x: float, y: float, z: float) -> None:
        self.t.append(t)
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)


class UavDashboardNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("uav_dashboard")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", args.use_sim_time)
        self.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", value=args.use_sim_time)]
        )

        self.start_time = time.monotonic()
        self.lock = threading.Lock()

        self.odom = DataSeries()
        self.odom_vel = DataSeries()
        self.gt = DataSeries()
        self.gt_vel = DataSeries()
        self.att = DataSeries()
        self.sensors = DataSeries()

        self.last_baro: Optional[float] = None
        self.last_mag: Optional[MagneticField] = None
        self.last_imu: Optional[Imu] = None
        self.last_stats: Optional[DiagnosticArray] = None

        self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)
        self.create_subscription(
            PoseStamped, args.gt_pose_topic, self.on_gt_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            TwistStamped, args.gt_twist_topic, self.on_gt_twist, qos_profile_sensor_data
        )
        self.create_subscription(Imu, args.imu_topic, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(
            MagneticField, args.mag_topic, self.on_mag, qos_profile_sensor_data
        )
        self.create_subscription(
            FluidPressure, args.baro_topic, self.on_baro, qos_profile_sensor_data
        )
        self.create_subscription(
            DiagnosticArray, args.stats_topic, self.on_stats, qos_profile_sensor_data
        )

    def _now(self) -> float:
        return time.monotonic() - self.start_time

    def on_odom(self, msg: Odometry) -> None:
        with self.lock:
            t = self._now()
            self.odom.append(
                t,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            )
            self.odom_vel.append(
                t,
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            )

    def on_gt_pose(self, msg: PoseStamped) -> None:
        with self.lock:
            t = self._now()
            self.gt.append(t, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def on_gt_twist(self, msg: TwistStamped) -> None:
        with self.lock:
            t = self._now()
            self.gt_vel.append(
                t, msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z
            )

    def on_imu(self, msg: Imu) -> None:
        with self.lock:
            self.last_imu = msg
            roll, pitch, yaw = quat_to_rpy(msg.orientation)
            self.att.append(self._now(), roll, pitch, yaw)

    def on_mag(self, msg: MagneticField) -> None:
        with self.lock:
            self.last_mag = msg

    def on_baro(self, msg: FluidPressure) -> None:
        with self.lock:
            self.last_baro = msg.fluid_pressure
            mag = mag_norm(self.last_mag)
            acc = imu_accel_norm(self.last_imu)
            self.sensors.append(self._now(), self.last_baro or 0.0, mag, acc)

    def on_stats(self, msg: DiagnosticArray) -> None:
        with self.lock:
            self.last_stats = msg


def quat_to_rpy(q) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def mag_norm(msg: Optional[MagneticField]) -> float:
    if msg is None:
        return 0.0
    return math.sqrt(
        msg.magnetic_field.x ** 2
        + msg.magnetic_field.y ** 2
        + msg.magnetic_field.z ** 2
    )


def imu_accel_norm(msg: Optional[Imu]) -> float:
    if msg is None:
        return 0.0
    return math.sqrt(
        msg.linear_acceleration.x ** 2
        + msg.linear_acceleration.y ** 2
        + msg.linear_acceleration.z ** 2
    )


class DashboardUI:
    def __init__(self, node: UavDashboardNode) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title("UAV Tunnel Dashboard")

        fig = Figure(figsize=(10, 7), dpi=100)
        self.ax_traj = fig.add_subplot(2, 2, 1)
        self.ax_vel = fig.add_subplot(2, 2, 2)
        self.ax_att = fig.add_subplot(2, 2, 3)
        self.ax_sens = fig.add_subplot(2, 2, 4)

        self.canvas = FigureCanvasTkAgg(fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.status = tk.StringVar(value="Waiting for data...")
        status_label = tk.Label(self.root, textvariable=self.status, anchor="w")
        status_label.pack(fill=tk.X)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def on_close(self) -> None:
        self.root.quit()
        self.root.destroy()

    def update(self) -> None:
        with self.node.lock:
            self.ax_traj.clear()
            self.ax_traj.set_title("Trajectory (Top-Down XY)")
            self.ax_traj.set_xlabel("X (m)")
            self.ax_traj.set_ylabel("Y (m)")
            self.ax_traj.set_aspect("equal", adjustable="box")
            self.ax_traj.grid(True, linestyle="--", alpha=0.3)
            if self.node.odom.x:
                self.ax_traj.plot(self.node.odom.x, self.node.odom.y, label="odom")
                self.ax_traj.scatter(
                    [self.node.odom.x[0], self.node.odom.x[-1]],
                    [self.node.odom.y[0], self.node.odom.y[-1]],
                    s=20,
                    marker="o",
                    label="odom start/end",
                )
            if self.node.gt.x:
                self.ax_traj.plot(self.node.gt.x, self.node.gt.y, label="ground truth")
                self.ax_traj.scatter(
                    [self.node.gt.x[0], self.node.gt.x[-1]],
                    [self.node.gt.y[0], self.node.gt.y[-1]],
                    s=20,
                    marker="x",
                    label="gt start/end",
                )
            self.ax_traj.legend(loc="best")

            self.ax_vel.clear()
            self.ax_vel.set_title("Velocity")
            if self.node.odom_vel.t:
                self.ax_vel.plot(self.node.odom_vel.t, self.node.odom_vel.x, label="vx")
                self.ax_vel.plot(self.node.odom_vel.t, self.node.odom_vel.y, label="vy")
                self.ax_vel.plot(self.node.odom_vel.t, self.node.odom_vel.z, label="vz")
            if self.node.gt_vel.t:
                self.ax_vel.plot(self.node.gt_vel.t, self.node.gt_vel.x, "--", label="gt vx")
                self.ax_vel.plot(self.node.gt_vel.t, self.node.gt_vel.y, "--", label="gt vy")
                self.ax_vel.plot(self.node.gt_vel.t, self.node.gt_vel.z, "--", label="gt vz")
            self.ax_vel.legend(loc="best")

            self.ax_att.clear()
            self.ax_att.set_title("Attitude (rad)")
            if self.node.att.t:
                self.ax_att.plot(self.node.att.t, self.node.att.x, label="roll")
                self.ax_att.plot(self.node.att.t, self.node.att.y, label="pitch")
                self.ax_att.plot(self.node.att.t, self.node.att.z, label="yaw")
            self.ax_att.legend(loc="best")

            self.ax_sens.clear()
            self.ax_sens.set_title("Sensors")
            if self.node.sensors.t:
                self.ax_sens.plot(self.node.sensors.t, self.node.sensors.x, label="baro")
                self.ax_sens.plot(self.node.sensors.t, self.node.sensors.y, label="mag| |")
                self.ax_sens.plot(self.node.sensors.t, self.node.sensors.z, label="accel| |")
            self.ax_sens.legend(loc="best")

            if self.node.last_stats:
                self.status.set("Stats OK")
            elif self.node.odom.t:
                self.status.set("Odom OK (no stats)")
            else:
                self.status.set("Waiting for odometry")

        self.canvas.draw_idle()
        self.root.after(200, self.update)

    def run(self) -> None:
        self.root.after(200, self.update)
        self.root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description="UAV tunnel GUI dashboard")
    parser.add_argument("--use-sim-time", action="store_true", default=False)
    parser.add_argument("--odom-topic", default="/uav0/basic_odom")
    parser.add_argument("--stats-topic", default="/uav0/basic_odom/stats")
    parser.add_argument("--gt-pose-topic", default="/uav0/ground_truth/pose")
    parser.add_argument("--gt-twist-topic", default="/uav0/ground_truth/twist")
    parser.add_argument("--imu-topic", default="/uav0/sensor_measurements/imu")
    parser.add_argument("--mag-topic", default="/uav0/sensor_measurements/magnetometer")
    parser.add_argument("--baro-topic", default="/uav0/sensor_measurements/air_pressure")
    args = parser.parse_args()

    rclpy.init()
    node = UavDashboardNode(args)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    ui = DashboardUI(node)
    try:
        ui.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
