import rclpy
from rclpy.node import Node


class HelloRobotNode(Node):

    def __init__(self):
        super().__init__('hello_robot_node')
        self.get_logger().info('Hello!My first ROS2 node is running.')


def main(args=None):
    rclpy.init(args=args)

    node = HelloRobotNode()

    rclpy.spin(node)

    node.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()