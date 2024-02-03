#!/env/bin/env python3

import rclpy
import json
from rclpy.node import Node
from spot_msgs.msg import BatteryStateArray
from spot_msgs.msg import String  # Import String message type for the new topic

class BatteryLevelSubscriberNode(Node):

    def __init__(self):
        super().__init__("battery_Level_subscriber")
        self.battery_level_subscriber_ = self.create_subscription(
            BatteryStateArray, "/status/battery_states", self.battery_states_callback, 10)
        self.battery_state_publisher_ = self.create_publisher(String, "/custom_topic/battery_state", 10)

    def battery_states_callback(self, msg):
        # Process the received battery level data as needed
        for battery_state in msg.battery_states:
            battery_state_dict = {
                'charge_percentage': battery_state.charge_percentage,
            }
        self.get_logger().info(json.dumps(battery_state_dict, indent=2)) 
            # Publish battery_state_dict to a new topic
        self.publish_battery_state(battery_state_dict)

    def publish_battery_state(self, battery_state_dict):
        msg = String()
        msg.data = json.dumps(battery_state_dict)
        self.battery_state_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryLevelSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
