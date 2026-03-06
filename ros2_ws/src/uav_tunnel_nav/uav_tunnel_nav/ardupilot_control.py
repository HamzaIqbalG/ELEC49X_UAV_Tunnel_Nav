"""ArduPilot SITL flight controller for GPS-denied tunnel navigation.

Connects to ArduPilot SITL via MAVLink (pymavlink), manages the flight
state machine (arm -> takeoff -> navigate), and bridges ROS 2 /cmd_vel
velocity commands to ArduPilot body-frame setpoints using GUIDED_NOGPS mode.
"""

import threading
import time
from enum import Enum, auto
import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from pymavlink import mavutil


class FlightPhase(Enum):
    CONNECTING = auto()
    WAITING_FOR_EKF = auto()
    SETTING_MODE = auto()
    ARMING = auto()
    TAKING_OFF = auto()
    NAVIGATING = auto()


class ArduPilotControl(Node):
    """Bridges ROS 2 cmd_vel commands to ArduPilot MAVLink velocity setpoints."""

    GUIDED_MODE = 4  # Phase 1: GPS-based GUIDED; Phase 2 will switch to GUIDED_NOGPS (20)
    # SET_POSITION_TARGET_LOCAL_NED type_mask:
    #   ignore position (bits 0-2), use velocity (bits 3-5 clear),
    #   ignore accel (bits 6-8), ignore yaw (bit 10), use yaw_rate (bit 11 clear)
    VELOCITY_TYPEMASK = 0b0000_0101_1100_0111  # 1479

    def __init__(self) -> None:
        super().__init__("ardupilot_control")

        self.declare_parameter("connection", "tcp:127.0.0.1:5760")
        self.declare_parameter("target_altitude", 1.0)
        self.declare_parameter("takeoff_speed", 0.5)
        self.declare_parameter("takeoff_tolerance", 0.15)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("cmd_vel_timeout", 1.0)
        self.declare_parameter("ekf_wait_s", 10.0)

        self._conn_str = self._param_str("connection")
        self._target_alt = self._param_float("target_altitude")
        self._takeoff_speed = self._param_float("takeoff_speed")
        self._takeoff_tol = self._param_float("takeoff_tolerance")
        self._cmd_vel_timeout = self._param_float("cmd_vel_timeout")
        self._ekf_wait = self._param_float("ekf_wait_s")
        rate = self._param_float("control_rate")

        self._mav = None
        self._phase = FlightPhase.CONNECTING
        self._armed = False
        self._mode = ""
        self._rel_alt = 0.0
        self._phase_entered = time.monotonic()
        self._last_heartbeat_sent = 0.0
        self._last_statustext = ""
        self._takeoff_cmd_sent = False

        self._last_cmd = Twist()
        self._last_cmd_time = 0.0
        self._cmd_lock = threading.Lock()

        self._state_pub = self.create_publisher(String, "/ardupilot/state", 10)
        self.create_subscription(
            Twist, self._param_str("cmd_vel_topic"), self._on_cmd_vel, 10
        )

        # Background thread for MAVLink connect (can block on TCP handshake)
        self._connect_thread = threading.Thread(
            target=self._connect_loop, daemon=True
        )
        self._connect_thread.start()

        self._log_path = Path("/ros2_ws/src/uav_tunnel_nav/debug-967540.log")

        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"ArduPilot control starting (target: {self._conn_str})"
        )

    # #region agent log
    def _agent_log(self, hypothesis_id: str, msg: str, data: dict) -> None:
        """Lightweight NDJSON logger for debug mode."""
        try:
            payload = {
                "sessionId": "967540",
                "runId": "pre-fix",
                "hypothesisId": hypothesis_id,
                "location": "ardupilot_control.py",
                "message": msg,
                "data": data,
                "timestamp": int(time.time() * 1000),
            }
            with self._log_path.open("a") as f:
                f.write(json.dumps(payload) + "\n")
        except Exception:
            # Never let logging break control loop
            pass
    # #endregion

    # ── helpers ──────────────────────────────────────────────────────

    def _param_str(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _param_float(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _set_phase(self, phase: FlightPhase) -> None:
        if phase != self._phase:
            self.get_logger().info(f"Phase: {self._phase.name} -> {phase.name}")
            self._phase = phase
            self._phase_entered = time.monotonic()
            if phase == FlightPhase.TAKING_OFF:
                self._takeoff_cmd_sent = False

    def _phase_age(self) -> float:
        return time.monotonic() - self._phase_entered

    # ── MAVLink connection (runs in background thread) ───────────────

    def _connect_loop(self) -> None:
        while rclpy.ok() and self._mav is None:
            try:
                self.get_logger().info(
                    f"Connecting to ArduPilot at {self._conn_str} ..."
                )
                conn = mavutil.mavlink_connection(self._conn_str)
                conn.wait_heartbeat(timeout=30)
                self._mav = conn
                self.get_logger().info(
                    f"Heartbeat received (sysid={conn.target_system} "
                    f"compid={conn.target_component})"
                )
                self._request_data_streams()
                self._set_phase(FlightPhase.WAITING_FOR_EKF)
            except Exception as e:
                self.get_logger().warn(f"Connect failed: {e}  — retrying in 3 s")
                time.sleep(3)

    def _request_data_streams(self) -> None:
        """Ask ArduPilot to send us VFR_HUD and HEARTBEAT at reasonable rates."""
        m = self._mav
        for stream, rate in [
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 4),   # VFR_HUD
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 4),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2),
        ]:
            m.mav.request_data_stream_send(
                m.target_system, m.target_component, stream, rate, 1
            )

    # ── cmd_vel subscriber ───────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._cmd_lock:
            self._last_cmd = msg
            self._last_cmd_time = time.monotonic()

    # ── MAVLink send helpers ─────────────────────────────────────────

    def _send_heartbeat(self) -> None:
        now = time.monotonic()
        if now - self._last_heartbeat_sent < 1.0:
            return
        self._last_heartbeat_sent = now
        self._mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )

    def _send_set_mode(self, mode_id: int) -> None:
        self._mav.mav.command_long_send(
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
            0, 0, 0, 0, 0,
        )

    def _send_arm(self) -> None:
        self._mav.mav.command_long_send(
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # arm
            21196,  # force-arm magic number (bypasses pre-arm checks)
            0, 0, 0, 0, 0,
        )

    def _send_takeoff(self, alt_m: float) -> None:
        """Request a guided takeoff, like MAVProxy `takeoff <alt>`."""
        self._mav.mav.command_long_send(
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            alt_m,
        )

    def _send_velocity_ned(self, vx: float, vy: float, vz: float,
                           yaw_rate: float) -> None:
        """Send body-frame velocity. NED convention: +vx forward, +vy right, +vz down."""
        self._mav.mav.set_position_target_local_ned_send(
            0,
            self._mav.target_system,
            self._mav.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            self.VELOCITY_TYPEMASK,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, yaw_rate,
        )

    # ── receive & decode ─────────────────────────────────────────────

    def _drain_messages(self) -> None:
        while True:
            msg = self._mav.recv_match(blocking=False)
            if msg is None:
                break
            t = msg.get_type()
            if t == "HEARTBEAT":
                self._armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                _MODES = {
                    0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
                    5: "LOITER", 6: "RTL", 9: "LAND", 20: "GUIDED_NOGPS",
                }
                self._mode = _MODES.get(msg.custom_mode, f"MODE_{msg.custom_mode}")
                self._agent_log(
                    "H1",
                    "HEARTBEAT",
                    {
                        "mode": self._mode,
                        "armed": self._armed,
                        "phase": self._phase.name,
                    },
                )
            elif t == "GLOBAL_POSITION_INT":
                self._rel_alt = msg.relative_alt / 1000.0
                self._agent_log(
                    "H1",
                    "GLOBAL_POSITION_INT",
                    {
                        "rel_alt": self._rel_alt,
                        "phase": self._phase.name,
                    },
                )
            elif t == "STATUSTEXT":
                # Useful for catching pre-arm/failsafe reasons and unexpected disarms.
                txt = getattr(msg, "text", "").strip()
                if txt:
                    self._last_statustext = txt
                    # Severity: 0=EMERGENCY ... 7=DEBUG. ArduPilot uses 3/4 for important warnings.
                    sev = getattr(msg, "severity", 7)
                    if sev <= 4:
                        self.get_logger().warn(f"AP: {txt}")

    # ── main control tick (10 Hz) ────────────────────────────────────

    def _tick(self) -> None:
        if self._mav is None:
            self._publish_state()
            return

        try:
            self._drain_messages()
            self._send_heartbeat()
        except Exception as e:
            self.get_logger().warn(f"MAVLink recv error: {e}")
            return

        try:
            self._run_state_machine()
        except Exception as e:
            self.get_logger().error(f"Control error: {e}")

        self._publish_state()

    def _run_state_machine(self) -> None:
        phase = self._phase

        if phase == FlightPhase.WAITING_FOR_EKF:
            if self._phase_age() > self._ekf_wait:
                self.get_logger().info("EKF wait complete, setting GUIDED")
                self._set_phase(FlightPhase.SETTING_MODE)

        elif phase == FlightPhase.SETTING_MODE:
            if self._mode == "GUIDED":
                self.get_logger().info("GUIDED confirmed, arming")
                self._set_phase(FlightPhase.ARMING)
            elif self._phase_age() > 2.0:
                self._send_set_mode(self.GUIDED_MODE)
                self._phase_entered = time.monotonic()

        elif phase == FlightPhase.ARMING:
            if self._armed:
                self.get_logger().info("Armed — taking off")
                self._set_phase(FlightPhase.TAKING_OFF)
            elif self._phase_age() > 2.0:
                self._send_arm()
                self._phase_entered = time.monotonic()

        elif phase == FlightPhase.TAKING_OFF:
            if self._rel_alt >= self._target_alt - self._takeoff_tol:
                self.get_logger().info(
                    f"Altitude {self._rel_alt:.2f} m reached — navigating"
                )
                self._set_phase(FlightPhase.NAVIGATING)
            elif self._phase_age() > 30.0:
                self.get_logger().warn("Takeoff timeout — entering nav anyway")
                self._set_phase(FlightPhase.NAVIGATING)
            elif not self._takeoff_cmd_sent and self._phase_age() > 2.0:
                self.get_logger().info(
                    f"Sending NAV_TAKEOFF to {self._target_alt:.1f} m "
                    f"(rel_alt={self._rel_alt:.2f})"
                )
                self._send_takeoff(self._target_alt)
                self._takeoff_cmd_sent = True

        elif phase == FlightPhase.NAVIGATING:
            self._navigate()

    def _navigate(self) -> None:
        with self._cmd_lock:
            cmd = self._last_cmd
            age = time.monotonic() - self._last_cmd_time

        if age > self._cmd_vel_timeout:
            # No recent cmd_vel — hold position (zero velocity)
            self._send_velocity_ned(0, 0, 0, 0)
            return

        # ROS body frame (FLU): +x forward, +y left, +z up, +yaw CCW
        # ArduPilot body NED:   +vx forward, +vy right, +vz down, +yaw_rate CW
        self._send_velocity_ned(
            cmd.linear.x,
            -cmd.linear.y,
            -cmd.linear.z,
            -cmd.angular.z,
        )

    # ── telemetry ────────────────────────────────────────────────────

    def _publish_state(self) -> None:
        msg = String()
        msg.data = (
            f"{self._phase.name} | mode={self._mode} "
            f"armed={self._armed} alt={self._rel_alt:.2f}m"
            + (f" | AP: {self._last_statustext}" if self._last_statustext else "")
        )
        self._state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArduPilotControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
