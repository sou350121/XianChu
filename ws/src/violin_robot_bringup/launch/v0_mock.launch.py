from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="violin_robot_drivers",
                executable="driver_mock_node",
                name="driver_mock_node",
                output="screen",
                parameters=[
                    {
                        "joint_rate_hz": 100.0,
                        "pose_rate_hz": 30.0,
                        "tactile_rate_hz": 50.0,
                        "audio_rate_hz": 20.0,
                        "slip_every_s": 12.0,
                        "slip_duration_s": 2.5,
                    }
                ],
            ),
            Node(
                package="violin_robot_music_spec",
                executable="music_spec_node",
                name="music_spec_node",
                output="screen",
                parameters=[{"publish_once": True, "republish_period_s": 0.0}],
            ),
            Node(
                package="violin_robot_policy",
                executable="policy_stub_node",
                name="policy_stub_node",
                output="screen",
                parameters=[{"publish_rate_hz": 20.0}],
            ),
            Node(
                package="violin_robot_control",
                executable="tactile_guard_node",
                name="tactile_guard_node",
                output="screen",
                parameters=[
                    {
                        "slip_risk_estop_threshold": 0.95,
                        "slip_risk_slowdown_threshold": 0.6,
                        "slowdown_factor": 0.4,
                    }
                ],
            ),
        ]
    )

