import os
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from doghead_interfaces.msg import EndEffectorReference, DriverSessionState
from doghead_motion.json_trajectory_loader import (
    JsonTrajectoryLoader,
    CoordinateTransformConfig,
    TrajectoryPoint,
)


class DogheadTrajectoryPlayerNode(Node):
    """
    ROS 2 node that reads a calligraphy toolpath JSON file and sequentially
    publishes EndEffectorReference messages to the motion solver layer (/doghead/motion/ef_reference).
    Preserves PVA structure (position, linear_velocity, linear_acceleration).
    """

    def __init__(self):
        super().__init__('doghead_trajectory_player_node')

        # 1. Parameter declarations
        self.declare_parameter('file_path', '')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('unit', 'mm')
        self.declare_parameter('scale_x', 1.0)
        self.declare_parameter('scale_y', 1.0)
        self.declare_parameter('scale_z', 1.0)
        self.declare_parameter('offset_x', 0.0)
        self.declare_parameter('offset_y', 0.0)
        self.declare_parameter('offset_z', 0.0)
        self.declare_parameter('playback_rate_hz', 10.0)  # Waypoint feed rate
        self.declare_parameter('loop', False)

        # 2. Retrieve parameters
        self.file_path = self.get_parameter('file_path').value
        self.auto_start = self.get_parameter('auto_start').value
        self.playback_rate_hz = float(self.get_parameter('playback_rate_hz').value)
        self.loop = self.get_parameter('loop').value

        if not math.isfinite(self.playback_rate_hz) or self.playback_rate_hz <= 0.0:
            raise ValueError('playback_rate_hz must be positive and finite')

        self.transform_config = CoordinateTransformConfig(
            unit=self.get_parameter('unit').value,
            scale_x=float(self.get_parameter('scale_x').value),
            scale_y=float(self.get_parameter('scale_y').value),
            scale_z=float(self.get_parameter('scale_z').value),
            offset_x=float(self.get_parameter('offset_x').value),
            offset_y=float(self.get_parameter('offset_y').value),
            offset_z=float(self.get_parameter('offset_z').value),
        )

        self.loader = JsonTrajectoryLoader(self.transform_config)
        self.waypoints: list[TrajectoryPoint] = []
        self.current_index = 0
        self.state = "IDLE"  # "IDLE", "PLAYING", "PAUSED", "COMPLETED", "FAULT"
        self.driver_armed = False

        # 3. Publishers
        self.ef_pub = self.create_publisher(EndEffectorReference, '/doghead/motion/ef_reference', 10)
        self.status_pub = self.create_publisher(String, '/doghead/trajectory_player/status', 10)

        # 4. Subscriptions
        self.create_subscription(
            DriverSessionState, '/doghead/driver/session_state', self._on_driver_session_state, 10
        )
        self.create_subscription(
            String, '/doghead/trajectory_player/command', self._on_player_command, 10
        )
        self.create_subscription(
            String, '/doghead/motion/status', self._on_motion_status, 10
        )

        # 5. Playback Timer
        self.timer_period = 1.0 / self.playback_rate_hz
        self.playback_timer = self.create_timer(self.timer_period, self._playback_step)

        # 6. Status heartbeat timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info("Doghead Trajectory Player Node initialized.")

        # Load file if provided at startup
        if self.file_path:
            self.load_file(self.file_path)
            if self.auto_start:
                self.play()

    def load_file(self, file_path: str) -> bool:
        """Load and parse waypoints from a JSON file."""
        if not os.path.isfile(file_path):
            self.get_logger().error(f"Trajectory file not found: '{file_path}'")
            self.state = "FAULT"
            return False

        try:
            self.waypoints = self.loader.load_from_file(file_path, self.transform_config)
            self.current_index = 0
            self.state = "IDLE"
            self.file_path = file_path
            self.get_logger().info(
                f"Successfully loaded {len(self.waypoints)} waypoints from '{file_path}'"
            )
            self._publish_status()
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to load trajectory file '{file_path}': {e}")
            self.state = "FAULT"
            self.waypoints = []
            self.current_index = 0
            self._publish_status()
            return False

    def load_json_string(self, json_str: str) -> bool:
        """Load and parse waypoints directly from a JSON string."""
        try:
            self.waypoints = self.loader.load_from_json_string(json_str, self.transform_config)
            self.current_index = 0
            self.state = "IDLE"
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from string")
            self._publish_status()
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to parse trajectory JSON string: {e}")
            self.state = "FAULT"
            self.waypoints = []
            self.current_index = 0
            self._publish_status()
            return False

    def play(self):
        """Start or resume playback."""
        if not self.waypoints:
            self.get_logger().warn("Cannot play: No waypoints loaded.")
            return
        if self.current_index >= len(self.waypoints):
            self.current_index = 0
        self.state = "PLAYING"
        self.get_logger().info(f"Trajectory playback started at waypoint [{self.current_index + 1}/{len(self.waypoints)}]")
        self._publish_status()

    def pause(self):
        """Pause playback."""
        if self.state == "PLAYING":
            self.state = "PAUSED"
            self.get_logger().info(f"Trajectory playback paused at waypoint [{self.current_index}/{len(self.waypoints)}]")
            self._publish_status()

    def stop(self):
        """Stop playback and reset to start."""
        self.state = "IDLE"
        self.current_index = 0
        self.get_logger().info("Trajectory playback stopped.")
        self._publish_status()

    def _playback_step(self):
        """Timer callback to step through and publish trajectory waypoints."""
        if self.state != "PLAYING" or not self.waypoints:
            return

        if self.current_index >= len(self.waypoints):
            if self.loop:
                self.current_index = 0
            else:
                self.state = "COMPLETED"
                self.get_logger().info("Trajectory playback COMPLETED.")
                self._publish_status()
                return

        pt = self.waypoints[self.current_index]

        # Construct EndEffectorReference message
        msg = EndEffectorReference()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # mode: 0 for travel, 1 for write
        msg.mode = 1 if pt.is_writing else 0

        # Position (P)
        msg.position.x = float(pt.x)
        msg.position.y = float(pt.y)
        msg.position.z = float(pt.z)

        # Velocity (V) - preserved
        msg.linear_velocity.x = float(pt.vx)
        msg.linear_velocity.y = float(pt.vy)
        msg.linear_velocity.z = float(pt.vz)

        # Acceleration (A) - preserved
        msg.linear_acceleration.x = float(pt.ax)
        msg.linear_acceleration.y = float(pt.ay)
        msg.linear_acceleration.z = float(pt.az)

        self.ef_pub.publish(msg)

        self.current_index += 1

    def _on_driver_session_state(self, msg: DriverSessionState):
        """Track driver arming state."""
        self.driver_armed = msg.armed
        if not msg.armed and self.state == "PLAYING":
            self.get_logger().warn(f"Driver disarmed ('{msg.disarm_reason}'). Pausing trajectory player.")
            self.pause()

    def _on_motion_status(self, msg: String):
        """Monitor motion node status for rejections."""
        if "REJECTED" in msg.data:
            self.get_logger().warn(f"Motion Node reported rejection: {msg.data}")

    def _on_player_command(self, msg: String):
        """Handle command strings: 'play', 'pause', 'stop', 'load <path>'."""
        cmd = msg.data.strip()
        cmd_lower = cmd.lower()

        if cmd_lower == "play":
            self.play()
        elif cmd_lower == "pause":
            self.pause()
        elif cmd_lower == "stop":
            self.stop()
        elif cmd_lower.startswith("load "):
            file_path = cmd[5:].strip()
            self.load_file(file_path)
        else:
            self.get_logger().warn(f"Unknown player command: '{cmd}'")

    def _publish_status(self):
        """Publish status message."""
        total = len(self.waypoints)
        idx = self.current_index
        status_str = (
            f"State: {self.state} | "
            f"Progress: [{idx}/{total}] | "
            f"Armed: {self.driver_armed} | "
            f"File: '{self.file_path}'"
        )
        msg = String()
        msg.data = status_str
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DogheadTrajectoryPlayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

