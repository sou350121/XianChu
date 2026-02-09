import json
import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


@dataclass
class MockConfig:
    joint_rate_hz: float = 100.0
    pose_rate_hz: float = 30.0
    tactile_rate_hz: float = 50.0
    audio_rate_hz: float = 20.0
    slip_every_s: float = 12.0
    slip_duration_s: float = 2.5


class DriverMockNode(Node):
    """
    v0 vertical slice mock driver.

    Publishes:
    - /violin/state/joint_state (sensor_msgs/JointState)
    - /violin/state/end_effector_pose (geometry_msgs/PoseStamped)
    - /violin/tactile/features (std_msgs/String JSON)
    - /violin/audio/features (std_msgs/String JSON)
    """

    def __init__(self) -> None:
        super().__init__("driver_mock_node")

        self.declare_parameter("joint_rate_hz", 100.0)
        self.declare_parameter("pose_rate_hz", 30.0)
        self.declare_parameter("tactile_rate_hz", 50.0)
        self.declare_parameter("audio_rate_hz", 20.0)
        self.declare_parameter("slip_every_s", 12.0)
        self.declare_parameter("slip_duration_s", 2.5)

        self.cfg = MockConfig(
            joint_rate_hz=float(self.get_parameter("joint_rate_hz").value),
            pose_rate_hz=float(self.get_parameter("pose_rate_hz").value),
            tactile_rate_hz=float(self.get_parameter("tactile_rate_hz").value),
            audio_rate_hz=float(self.get_parameter("audio_rate_hz").value),
            slip_every_s=float(self.get_parameter("slip_every_s").value),
            slip_duration_s=float(self.get_parameter("slip_duration_s").value),
        )

        self.pub_joint = self.create_publisher(JointState, "/violin/state/joint_state", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/violin/state/end_effector_pose", 10)
        self.pub_tactile = self.create_publisher(String, "/violin/tactile/features", 10)
        self.pub_audio = self.create_publisher(String, "/violin/audio/features", 10)

        # Optional: listen to e-stop to reflect state in mock features
        self.estop = False
        self.create_subscription(Bool, "/violin/control/emergency_stop", self._on_estop, 10)

        self._t0 = time.time()

        self.create_timer(1.0 / self.cfg.joint_rate_hz, self._tick_joint)
        self.create_timer(1.0 / self.cfg.pose_rate_hz, self._tick_pose)
        self.create_timer(1.0 / self.cfg.tactile_rate_hz, self._tick_tactile)
        self.create_timer(1.0 / self.cfg.audio_rate_hz, self._tick_audio)

        self.get_logger().info("driver_mock_node started (v0 slice).")

    def _on_estop(self, msg: Bool) -> None:
        self.estop = bool(msg.data)

    def _now_float(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _tick_joint(self) -> None:
        t = self._now_float()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["left_joint_1", "right_joint_1"]
        msg.position = [0.3 * math.sin(t), 0.3 * math.cos(t)]
        msg.velocity = [0.3 * math.cos(t), -0.3 * math.sin(t)]
        msg.effort = [0.0, 0.0]
        self.pub_joint.publish(msg)

    def _tick_pose(self) -> None:
        t = self._now_float()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = 0.5 + 0.02 * math.sin(t)
        msg.pose.position.y = 0.1 + 0.02 * math.cos(t)
        msg.pose.position.z = 1.0
        msg.pose.orientation.w = 1.0
        self.pub_pose.publish(msg)

    def _slip_phase(self, wall_t: float) -> float:
        if self.cfg.slip_every_s <= 0:
            return 0.0
        p = wall_t % self.cfg.slip_every_s
        if p < self.cfg.slip_duration_s:
            return 1.0 - (p / max(self.cfg.slip_duration_s, 1e-6)) * 0.2
        return 0.0

    def _tick_tactile(self) -> None:
        wall_t = time.time() - self._t0
        slip_risk = self._slip_phase(wall_t)
        contact_phase = "in_contact" if wall_t > 3.0 else "pre_contact"
        if self.estop:
            contact_phase = "e_stop"
            slip_risk = 0.0

        # v0 tactile features follow contracts/data_schema/episode_schema.md
        sources = {
            # Most runs: simulate bow_string slip risk to trigger slowdown / recovery.
            "bow_string": {
                "contact_event": contact_phase in ("pre_contact", "in_contact"),
                "slip_risk": float(slip_risk),
                "contact_phase": contact_phase,
                "contact_mode": "preslip" if slip_risk > 0.4 else "stick",
                "hf_energy": 0.02 + 0.2 * float(slip_risk),
                "bow_normal_force_est": 0.8,
            },
            # Keep violin_grip stable in mock.
            "violin_grip": {
                "contact_event": True,
                "slip_risk": 0.02,
                "contact_phase": "in_contact",
                "contact_mode": "stick",
                "hf_energy": 0.0,
            },
            "bow_grip": {
                "contact_event": True,
                "slip_risk": 0.01,
                "contact_phase": "in_contact",
                "contact_mode": "stick",
                "hf_energy": 0.0,
            },
            "finger_string": {
                "contact_event": False,
                "slip_risk": 0.0,
                "contact_phase": "free_space",
                "contact_mode": "unknown",
                "hf_energy": 0.0,
            },
        }

        payload = {
            "timestamp": self._now_float(),
            "sources": sources,
            # Backward compatible flat fields (older consumers may rely on these).
            "contact_event": sources["bow_string"]["contact_event"],
            "slip_risk": sources["bow_string"]["slip_risk"],
            "contact_phase": sources["bow_string"]["contact_phase"],
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_tactile.publish(msg)

    def _tick_audio(self) -> None:
        t = self._now_float()
        payload = {
            "timestamp": t,
            "f0_hz": 440.0 + 5.0 * math.sin(2 * math.pi * 0.5 * t),
            "bpm_est": 90.0,
            "energy": 0.5 + 0.1 * math.sin(2 * math.pi * 1.0 * t),
            "noise_ratio": 0.05 + 0.02 * math.sin(2 * math.pi * 0.3 * t),
            "valid": True,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_audio.publish(msg)


def main() -> None:
    rclpy.init()
    node = DriverMockNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

