#!/usr/bin/env python3

from random import randint
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class GridMapPublisher(Node):
    def __init__(self, map_length: float = 100.0, resolution: float = 0.1):
        super().__init__("grid_map_publisher")
        self.publisher_ = self.create_publisher(OccupancyGrid, "~/output/heatmap", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.width = int(map_length / resolution)
        self.height = int(map_length / resolution)
        self.resolution = resolution
        self.map_length = map_length

    def timer_callback(self):
        msg = OccupancyGrid()
        msg.header.frame_id = "/map"
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.resolution = self.resolution
        msg.info.origin.position.x = -self.map_length / 2
        msg.info.origin.position.y = -self.map_length / 2
        msg.info.origin.orientation.w = 1.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        for i in range(self.width):
            for j in range(self.height):
                msg.data.append(randint(0, 100))

        self.publisher_.publish(msg)
        self.get_logger().info("Published")


def main(args=None):
    rclpy.init(args=args)
    pub = GridMapPublisher()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

