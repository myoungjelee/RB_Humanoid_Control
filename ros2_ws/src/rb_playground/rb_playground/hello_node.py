import rclpy
from rclpy.node import Node


class HelloNode(Node):
    def __init__(self):
        super().__init__("hello_node")
        self.get_logger().info("hello_node started")


def main():
    rclpy.init()
    node = HelloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
