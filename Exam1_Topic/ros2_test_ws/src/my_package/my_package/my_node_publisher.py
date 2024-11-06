import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisherNode(Node):
    def __init__(self):
        super().__init__("simple_publisher_node")
        self.publisher_ = self.create_publisher(
            String, "simple_topic", 1
        )  # Topic name and queue size
        self.timer = self.create_timer(
            1.0, self.timer_callback
        )  # Timer callback every 1 second

    def timer_callback(self):
        msg = String()
        msg.data = "Hello, ROS 2!"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisherNode()

    try:
        rclpy.spin(node)  # Keeps the node running to listen for messages
    except KeyboardInterrupt:
        pass

    print("Shutdown.")
    node.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
