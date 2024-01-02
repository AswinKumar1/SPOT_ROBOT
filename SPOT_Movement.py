#!/env/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class SpotMovementController(Node):
  def __init__(self):
    self.command_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)

  def send_velocity_command(self, linear_x, angular_z):
    twist_msg = Twist()
    twist_msg.linear_x = linear_x
    twist_msg.angular_z = angular_z
    self.publisher_.publish(twist_msg)
    self.get_logger().info("Sent Velocity Command: linear={linear_x), angular={angular_z}")

def main(args=None)
  rclpy.init(args=args)
  node = SpotMovementController()
  try:
    node.send_velocity_command(target_x=1.0, target_y=0.0)
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass

if __name__ == '__main__'
  main()

