import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Joint order expected by arm_controller (ForwardCommandController)
ARM_JOINTS = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper',
]


class ArmTeleopNode(Node):
    """Bridge leader arm JointState messages to ros2_control arm_controller commands.

    Subscribes to /joint_states (published by the leader arm's joint_state_broadcaster
    or so101_ros2_node), extracts the 6 arm joint positions in the correct order, and
    publishes them to /arm_controller/commands (Float64MultiArray).
    """

    def __init__(self):
        super().__init__('arm_teleop_node')

        self.declare_parameter('leader_joint_states_topic', 'leader_arm/joint_states')

        topic = self.get_parameter('leader_joint_states_topic').value

        self.joint_states_sub = self.create_subscription(
            JointState, topic, self._joint_states_callback, 10)

        self.arm_cmd_pub = self.create_publisher(
            Float64MultiArray, '/arm_controller/commands', 10)

    def _joint_states_callback(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))

        positions = []
        for joint in ARM_JOINTS:
            if joint not in name_to_pos:
                return  # wait until all joints are present
            positions.append(name_to_pos[joint])

        out = Float64MultiArray()
        out.data = positions
        self.arm_cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
