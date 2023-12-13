#!/env/bin/env python3

import rclpy
import json
from rclpy.node import Node
from spot_msgs.msg import BatteryStateArray

class BatteryLevelSubscriberNode(Node):

    def __init__(self):
        super().__init__("battery_Level_subscriber")
        self.battery_level_subscriber_ = self.create_subscription(
            BatteryStateArray, "/status/battery_states", self.battery_states_callback, 10)

    def battery_states_callback(self, msg: BatteryStateArray):
        for battery_state in msg.battery_states:
            battery_state_dict = {
                'charge_percentage': battery_state.charge_percentage,
            }
        self.get_logger().info(json.dumps(battery_state_dict, indent = 2))

def main(args=None):
    rclpy.init(args=args)
    node = BatteryLevelSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()


