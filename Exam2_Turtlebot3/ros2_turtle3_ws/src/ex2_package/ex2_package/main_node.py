import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 1)
        self.create_subscription(
            LaserScan, "scan", self.scan_callback, 10
        )  # buffer = 10
        self.get_logger().info("Obstacle Avoidance Node has been started.")

    def scan_callback(self, msg: LaserScan):
        move_cmd = Twist()

        # Parameters
        min_distance = 0.80  # Minimum distance to consider as "too close"
        fov_angle = math.radians(20)  # deg -> rad

        # Find the indexes corresponding to the FoV angles
        end_index = int((fov_angle / 2 - msg.angle_min) / msg.angle_increment)

        selected_laser_data = msg.ranges[0:end_index]
        selected_laser_data.extend(msg.ranges[-end_index:])

        # Check the laser scan data within the specified FoV
        min_range_in_fov = min(selected_laser_data)  # first half

        if min_range_in_fov < min_distance:  # If something is too close
            move_cmd.linear.x = 0.0  # Stop moving
            move_cmd.angular.z = 0.5  # Turn to avoid obstacle
        else:
            move_cmd.linear.x = 0.2  # Move forward
            move_cmd.angular.z = 0.0  # No turn
        self.publisher_.publish(move_cmd)


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance_node = ObstacleAvoidanceNode()

    rclpy.spin(obstacle_avoidance_node)

    obstacle_avoidance_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
