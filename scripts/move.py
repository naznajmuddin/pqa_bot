import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from math import sin


class MoveHandsNode(Node):
    def __init__(self):
        super().__init__("move_hands_node")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(1.0, self.publish_joint_states)

    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()

        # Set joint values to move the hands
        joint_state.name = ["hand_joint", "opposite_hand_joint"]
        joint_state.position = [
            sin(self.get_clock().now().nanoseconds / 1e9),
            -sin(self.get_clock().now().nanoseconds / 1e9),
        ]  # You can adjust the trajectory as needed

        self.pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = MoveHandsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
