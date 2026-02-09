import json
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def default_mock_spec() -> Dict[str, Any]:
    """A tiny music_spec that follows contracts/data_schema/music_spec.md."""

    # v0: a tiny "piece" with 4 note events.
    return {
        "music_spec_version": "0.1",
        "piece_id": "mock_piece_001",
        "time_base": "seconds",
        "tempo_bpm": 90,
        "time_signature": {"beats_per_bar": 4, "beat_unit": 4},
        "events": [
            {
                "event_id": "n1",
                "type": "note",
                "t_start": 0.0,
                "t_end": 1.0,
                "pitch_midi": 69,  # A4
                "string": "A",
                "finger": None,
                "bow": {
                    "bow_direction": "down",
                    "bowing_style": "detache",
                    "bow_speed": 0.35,
                    "bow_force": 0.8,
                },
                "dynamics": {"mark": "mf"},
            },
            {
                "event_id": "n2",
                "type": "note",
                "t_start": 1.0,
                "t_end": 2.0,
                "pitch_midi": 71,  # B4
                "string": "A",
                "finger": None,
                "bow": {
                    "bow_direction": "up",
                    "bowing_style": "detache",
                    "bow_speed": 0.35,
                    "bow_force": 0.8,
                },
                "dynamics": {"mark": "mf"},
            },
            {
                "event_id": "n3",
                "type": "note",
                "t_start": 2.0,
                "t_end": 3.0,
                "pitch_midi": 72,  # C5
                "string": "E",
                "finger": None,
                "bow": {
                    "bow_direction": "down",
                    "bowing_style": "detache",
                    "bow_speed": 0.35,
                    "bow_force": 0.8,
                },
                "dynamics": {"mark": "mf"},
            },
            {
                "event_id": "n4",
                "type": "note",
                "t_start": 3.0,
                "t_end": 4.0,
                "pitch_midi": 71,  # B4
                "string": "A",
                "finger": None,
                "bow": {
                    "bow_direction": "up",
                    "bowing_style": "detache",
                    "bow_speed": 0.35,
                    "bow_force": 0.8,
                },
                "dynamics": {"mark": "mf"},
            },
        ],
    }


class MusicSpecNode(Node):
    def __init__(self) -> None:
        super().__init__("music_spec_node")

        self.declare_parameter("publish_once", True)
        self.declare_parameter("republish_period_s", 0.0)

        self.pub = self.create_publisher(String, "/violin/music/spec", 10)

        publish_once = bool(self.get_parameter("publish_once").value)
        republish_period_s = float(self.get_parameter("republish_period_s").value)

        if publish_once:
            self._publish()

        if republish_period_s > 0:
            self.create_timer(republish_period_s, self._publish)

        self.get_logger().info("music_spec_node started (v0 mock spec).")

    def _publish(self) -> None:
        payload = default_mock_spec()
        payload["timestamp"] = self.get_clock().now().nanoseconds / 1e9
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MusicSpecNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

