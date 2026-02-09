import json
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class TactileGuardNode(Node):
    """
    v0 tactile guard: gates command_raw -> command, can raise e-stop.

    Subscribes:
    - /violin/control/command_raw (String JSON)
    - /violin/tactile/features (String JSON)

    Publishes:
    - /violin/control/command (String JSON)
    - /violin/control/emergency_stop (Bool, latched behavior in logic)
    """

    def __init__(self) -> None:
        super().__init__("tactile_guard_node")

        self.declare_parameter("slip_risk_estop_threshold", 0.95)
        self.declare_parameter("slip_risk_slowdown_threshold", 0.6)
        self.declare_parameter("slowdown_factor", 0.4)

        self.estop_latched = False
        self.latest_tactile: Optional[Dict[str, Any]] = None
        self.latest_cmd: Optional[Dict[str, Any]] = None

        self.pub_cmd = self.create_publisher(String, "/violin/control/command", 10)
        self.pub_estop = self.create_publisher(Bool, "/violin/control/emergency_stop", 10)

        self.create_subscription(String, "/violin/control/command_raw", self._on_cmd_raw, 10)
        self.create_subscription(String, "/violin/tactile/features", self._on_tactile, 10)

        self.create_timer(0.02, self._tick)  # 50 Hz gate loop
        self.get_logger().info("tactile_guard_node started (v0).")

    def _on_cmd_raw(self, msg: String) -> None:
        try:
            self.latest_cmd = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse command_raw JSON: {e}")

    def _on_tactile(self, msg: String) -> None:
        try:
            self.latest_tactile = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse tactile/features JSON: {e}")

    def _tick(self) -> None:
        if self.latest_cmd is None:
            return

        slip_estop_threshold = float(self.get_parameter("slip_risk_estop_threshold").value)
        slip_slow_threshold = float(self.get_parameter("slip_risk_slowdown_threshold").value)
        slowdown_factor = float(self.get_parameter("slowdown_factor").value)

        tactile = self.latest_tactile or {}

        # Backward compatible parsing:
        # - new schema: {timestamp, sources: {<name>: {slip_risk, contact_phase, ...}}}
        # - old schema: {timestamp, slip_risk, contact_phase, ...}
        sources = tactile.get("sources", {}) if isinstance(tactile.get("sources", {}), dict) else {}
        old_slip_risk = float(tactile.get("slip_risk", 0.0))
        old_contact_phase = str(tactile.get("contact_phase", "unknown"))

        def _src_slip(name: str) -> float:
            v = sources.get(name, {})
            if isinstance(v, dict):
                return float(v.get("slip_risk", 0.0))
            return 0.0

        def _src_phase(name: str) -> str:
            v = sources.get(name, {})
            if isinstance(v, dict):
                return str(v.get("contact_phase", "unknown"))
            return "unknown"

        # Safety-first rule: violin_grip slip risk is treated as highest severity.
        violin_grip_slip = _src_slip("violin_grip")
        violin_grip_phase = _src_phase("violin_grip")

        # Global (for slowdown) is the max slip risk across sources, falling back to old flat slip_risk.
        max_src_slip = max([old_slip_risk] + [float(v.get("slip_risk", 0.0)) for v in sources.values() if isinstance(v, dict)] + [0.0])
        slip_risk = max_src_slip

        # Pick a representative phase for logging/guard annotation.
        contact_phase = _src_phase("bow_string")
        if contact_phase == "unknown":
            contact_phase = old_contact_phase

        # Latch e-stop if high risk or explicitly in e_stop phase.
        if (violin_grip_slip >= slip_estop_threshold) or (violin_grip_phase == "e_stop") or (old_contact_phase == "e_stop"):
            if not self.estop_latched:
                self.get_logger().warn(f"E-STOP latched. slip_risk={slip_risk:.3f}, phase={contact_phase}")
            self.estop_latched = True

        if self.estop_latched:
            estop_msg = Bool()
            estop_msg.data = True
            self.pub_estop.publish(estop_msg)

            # Publish a safe command (zeroed).
            safe = dict(self.latest_cmd)
            safe["control_mode"] = "e_stop"
            if "hand_params" in safe and isinstance(safe["hand_params"], dict):
                safe["hand_params"] = dict(safe["hand_params"])
                safe["hand_params"]["bow_speed"] = 0.0
                safe["hand_params"]["bow_force"] = 0.0
                safe["hand_params"]["bow_direction"] = "stop"
            out = String()
            out.data = json.dumps(safe, ensure_ascii=False)
            self.pub_cmd.publish(out)
            return

        # Otherwise gate/modify command based on slip risk.
        cmd = dict(self.latest_cmd)
        cmd["guard"] = {"slip_risk": slip_risk, "contact_phase": contact_phase}

        if slip_risk >= slip_slow_threshold:
            hp = cmd.get("hand_params", {})
            if isinstance(hp, dict):
                hp = dict(hp)
                if "bow_speed" in hp:
                    hp["bow_speed"] = float(hp["bow_speed"]) * slowdown_factor
                if "bow_force" in hp:
                    hp["bow_force"] = float(hp["bow_force"]) * slowdown_factor
                cmd["hand_params"] = hp
                cmd["control_mode"] = "recovery"

        out = String()
        out.data = json.dumps(cmd, ensure_ascii=False)
        self.pub_cmd.publish(out)


def main() -> None:
    rclpy.init()
    node = TactileGuardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

