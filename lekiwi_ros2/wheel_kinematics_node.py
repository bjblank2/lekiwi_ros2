import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class WheelKinematicsNode(Node):
    """Convert /cmd_vel (Twist) to wheel velocity commands for ros2_control.

    3-wheel omni-directional kinematics: wheel_1=ID7 (front), wheel_2=ID8 (left),
    wheel_3=ID9 (right), arranged 120° apart.
    Publishes rad/s to /wheel_velocity_controller/commands.
    """

    def __init__(self):
        super().__init__('wheel_kinematics_node')

        self.declare_parameter('wheel_separation_x', 0.3)
        self.declare_parameter('wheel_separation_y', 0.3)
        self.declare_parameter('wheel_radius', 0.05)

        self.wheel_separation_x = self.get_parameter('wheel_separation_x').value
        self.wheel_separation_y = self.get_parameter('wheel_separation_y').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        self.wheel_cmd_pub = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_controller/commands', 10)

    def _cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        w1, w2, w3 = self._omni3_kinematics(vx, vy, omega)

        out = Float64MultiArray()
        out.data = [w1, w2, w3]
        self.wheel_cmd_pub.publish(out)

    def _omni3_kinematics(self, vx: float, vy: float, omega: float):
        """3-wheel omni kinematics — returns [w1, w2, w3] in rad/s."""
        R = (self.wheel_separation_x + self.wheel_separation_y) / 4.0
        r = self.wheel_radius

        w1 = (-math.sin(math.radians(60)) * vx + math.cos(math.radians(60)) * vy + R * omega) / r
        w2 = (-math.sin(math.radians(180)) * vx + math.cos(math.radians(180)) * vy + R * omega) / r
        w3 = (-math.sin(math.radians(300)) * vx + math.cos(math.radians(300)) * vy + R * omega) / r

        return w1, w2, w3


def main(args=None):
    rclpy.init(args=args)
    node = WheelKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
