#!/env/bin/env python3

import rclpy 
from rclpy.node import Node
from spot_msgs.msg import BatteryStateArray

class BatteryLevelPublisherNode(Node):

    def __init__(self):
        super().__init__("Battery_Level")
        self.battery_level_pub_ = self.create_publisher(BatteryStateArray, "/status/battery_states", 10)
        self.timer = self.create_timer(1.0, self.get_battery_level)
        self.get_logger().info("Battery level is being published")

        def get_battery_level(self):
            msg = BatteryStateArray()
            self.battery_level_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryLevelPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


