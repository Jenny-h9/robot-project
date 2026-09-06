import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MessagePublisher(Node):

    def __init__(self):
        super().__init__('message_publisher')

        self.publisher_ = self.create_publisher(
            String,
            '/robot_message',
            10
        )

        self.counter = 0

        self.timer = self.create_timer(
            1.0,
            self.timer_callback
        )

    def timer_callback(self):
        self.counter += 1

        msg = String()
        msg.data = f'Hello ROS2: {self.counter}'

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Publishing: {msg.data}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = MessagePublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ =='__main__':
    main()