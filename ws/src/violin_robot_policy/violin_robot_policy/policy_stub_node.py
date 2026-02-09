import json
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PolicyStubNode(Node):
    """
    v0 vertical slice policy stub.

    Subscribes:
    - /violin/music/spec (std_msgs/String JSON)
    Publishes:
    - /violin/control/command_raw (std_msgs/String JSON)
    """

    def __init__(self) -> None:
        super().__init__("policy_stub_node")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.pub = self.create_publisher(String, "/violin/control/command_raw", 10)
        self.create_subscription(String, "/violin/music/spec", self._on_spec, 10)

        self._spec: Optional[Dict[str, Any]] = None
        self._start_t = self.get_clock().now().nanoseconds / 1e9

        self.create_timer(1.0 / max(self.rate_hz, 1e-6), self._tick)
        self.get_logger().info("policy_stub_node started (v0).")

    def _on_spec(self, msg: String) -> None:
        try:
            self._spec = json.loads(msg.data)
            self._start_t = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info("Loaded music_spec.")
        except Exception as e:
            self.get_logger().error(f"Failed to parse music_spec JSON: {e}")

    def _event_time_to_seconds(self, t_value: Any) -> float:
        """Converts event time to seconds based on music_spec time_base.

        v0 supports both:
        - time_base=seconds: use float seconds
        - time_base=ticks: convert ticks using tempo_bpm and ticks_per_quarter
        """

        if not self._spec:
            return float(t_value or 0.0)

        time_base = str(self._spec.get("time_base", "seconds"))
        if time_base == "ticks":
            tempo_bpm = float(self._spec.get("tempo_bpm", 90.0))
            tpq = float(self._spec.get("ticks_per_quarter", 480))
            ticks = float(t_value or 0.0)
            sec_per_quarter = 60.0 / max(tempo_bpm, 1e-6)
            return (ticks / max(tpq, 1e-6)) * sec_per_quarter

        return float(t_value or 0.0)

    def _find_event(self, t_rel_s: float) -> Optional[Dict[str, Any]]:
        if not self._spec:
            return None
        events = self._spec.get("events", [])
        for ev in events:
            t0 = self._event_time_to_seconds(ev.get("t_start", 0.0))
            t1 = self._event_time_to_seconds(ev.get("t_end", 0.0))
            if t0 <= t_rel_s < t1:
                return ev
        return None

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if not self._spec:
            return

        t_rel_s = now - self._start_t
        ev = self._find_event(t_rel_s)
        if ev is None:
            # After the last event: stop motion.
            payload = {
                "timestamp": now,
                "control_mode": "free_space",
                "ee_delta": {
                    "left": {"dpos_m": [0.0, 0.0, 0.0], "drot_rad": [0.0, 0.0, 0.0]},
                    "right": {"dpos_m": [0.0, 0.0, 0.0], "drot_rad": [0.0, 0.0, 0.0]},
                },
                "hand_params": {"bow_speed": 0.0, "bow_force": 0.0, "bow_direction": "stop"},
                "music_context": {"t_rel_s": t_rel_s},
            }
        else:
            # Backward compatible parsing (old flat fields vs new nested fields)
            bow = ev.get("bow", {}) if isinstance(ev.get("bow", {}), dict) else {}
            bow_dir = bow.get("bow_direction", ev.get("bow_direction", "down"))
            bowing_style = bow.get("bowing_style", ev.get("bowing", "detache"))

            dyn = ev.get("dynamics", {"mark": "mf"})
            if isinstance(dyn, dict):
                dyn_mark = str(dyn.get("mark", "mf"))
            else:
                dyn_mark = str(dyn)

            bow_force = float(bow.get("bow_force", 0.6 if dyn_mark in ("f", "ff") else 0.4 if dyn_mark in ("mf", "mp") else 0.25))
            bow_speed = float(bow.get("bow_speed", 0.2 if bowing_style == "legato" else 0.35))

            payload = {
                "timestamp": now,
                "control_mode": "in_contact",
                # v0: no real motion, just placeholders for delta actions
                "ee_delta": {
                    "left": {"dpos_m": [0.0, 0.0, 0.0], "drot_rad": [0.0, 0.0, 0.0]},
                    "right": {"dpos_m": [0.0, 0.0, 0.0], "drot_rad": [0.0, 0.0, 0.0]},
                },
                "hand_params": {
                    "bow_speed": bow_speed,
                    "bow_force": bow_force,
                    "bow_direction": bow_dir,
                    "target_string": ev.get("string"),
                    "target_pitch_midi": ev.get("pitch_midi"),
                },
                "music_context": {"t_rel_s": t_rel_s, "event": ev},
            }

        out = String()
        out.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = PolicyStubNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

