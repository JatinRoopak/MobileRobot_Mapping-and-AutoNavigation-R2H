#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class Mymode(Node):
    def __init__(self):
        super().__init__("first_node")
        self.counter = 0
        self.create_timer(0.3, self.timer)

    def timer(self):
        self.get_logger().info("Hello again and again " + str(self.counter))
        self.counter += 1
        

def main(args=None):
    rclpy.init(args=args)
    node = Mymode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()