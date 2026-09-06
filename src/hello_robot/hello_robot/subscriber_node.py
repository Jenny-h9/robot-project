import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MessageSubscriber(Node):

    def __init__(self):
        super().__init__('message_subscriber')

        self.subscirption = self.create_subscription(
            String,
            '/robot_message',
            self.message_callback,
            10
        )

    def message_callback(self,msg):
        self.get_logger().info(
            f'Received: {msg.data}'
        )



def main(args=None):
    rclpy.init(args=args)

    node = MessageSubscriber()

    rclpy.spin(node)
    
    node.destory_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
