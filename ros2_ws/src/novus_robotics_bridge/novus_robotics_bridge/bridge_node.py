"""ROS 2 node bridging nav/arm/sensor topics to the advanced_robotics Python app.

Subscribes to odometry and publishes velocity commands for the AMR side;
republishes joint states for the arm side. Keeps ROS 2 specifics isolated
here so advanced_robotics.amr/arm stay testable without a ROS 2 runtime.
"""
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


class BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("novus_robotics_bridge")

        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._joint_state_pub = self.create_publisher(JointState, "/arm/joint_states", 10)

        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        self.get_logger().info("novus_robotics_bridge started")

    def _on_odom(self, msg: Odometry) -> None:
        # Hand off pose to advanced_robotics.amr.navigation.Navigator here.
        self.get_logger().debug(
            f"odom: x={msg.pose.pose.position.x:.2f} y={msg.pose.pose.position.y:.2f}"
        )

    def publish_cmd_vel(self, linear_mps: float, angular_radps: float) -> None:
        msg = Twist()
        msg.linear.x = linear_mps
        msg.angular.z = angular_radps
        self._cmd_vel_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
