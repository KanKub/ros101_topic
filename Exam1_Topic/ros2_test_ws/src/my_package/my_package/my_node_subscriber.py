import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriberNode(Node):
    def __init__(self):
        super().__init__("simple_subscriber_node")
        # Create a subscription to the topic 'topic_name'
        self.subscription = self.create_subscription(
            String,  # Message type
            "simple_topic",  # Topic name
            self.listener_callback,  # Callback function
            1,  # Queue size
        )

    def listener_callback(self, msg):
        # This callback is called whenever a message is received
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriberNode()

    try:
        rclpy.spin(node)  # Keeps the node running to listen for messages
    except KeyboardInterrupt:
        pass

    print("Shutdown.")
    node.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
