import math
import sys
import time
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, blocking_sit
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api import geometry_pb2

# Function to calculate the Haversine distance
def haversine(lat1, lon1, lat2, lon2):
    R = 6371.0  # Earth radius in kilometers
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c * 1000   # Convert to meters
    return distance

# Function to calculate the bearing
def calculate_bearing(lat1, lon1, lat2, lon2):
    dlon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.atan2(x, y)
    bearing = math.degrees(bearing)
    bearing = (bearing + 360) % 360
    return bearing

# Function to convert track angle to cardinal direction
def track_to_direction(track):
    directions = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 
                  'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW']
    idx = round(track / 22.5) % 16
    return directions[idx]

class SpotWrapper:
    def __init__(self, spot_init=True):
        self.sdk = None
        self.robot = None
        if spot_init:
            # Initialize Spot Robot if required
            self.sdk = bosdyn.client.create_standard_sdk('MoveSpot')
            self.robot = self.sdk.create_robot('192.168.1.10')
            self.robot.authenticate('admin', 'Cards@2023spot')
            self.robot.time_sync.wait_for_sync()
            lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
            lease = lease_client.take()
            lease_keep_alive = LeaseKeepAlive(lease_client)
            self.robot.power_on(timeout_sec=20)
            assert self.robot.is_powered_on(), "Robot failed to power on"

    def stand_spot(self):
        try:
            command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            blocking_stand(command_client, timeout_sec=15)
        except Exception as e:
            print(f"Error standing Spot robot: {e}")

    def sit_spot(self):
        try:
            command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            blocking_sit(command_client, timeout_sec=10)
        except Exception as e:
            print(f"Error sitting Spot robot: {e}")

    def move_spot(self, goal_x, goal_y, duration_s):
        try:
            max_linear_velocity = geometry_pb2.Vec2(x=0.5, y=0.0)
            max_angular_velocity = 0.0

            max_se2_velocity = geometry_pb2.SE2Velocity(linear=max_linear_velocity, angular=max_angular_velocity)

            velocity_limit = geometry_pb2.SE2VelocityLimit(
                max_vel=max_se2_velocity
            )

            mobility_params = spot_command_pb2.MobilityParams(
                locomotion_hint=spot_command_pb2.HINT_AUTO,
                vel_limit=velocity_limit
            )

            frame_tree_snapshot = self.robot.get_frame_tree_snapshot()
            command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            command = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(goal_x_rt_body=goal_x, goal_y_rt_body=goal_y, goal_heading_rt_body=0.0, frame_tree_snapshot=frame_tree_snapshot, params=mobility_params, body_height=0.0, build_on_command=None)
            command_client.robot_command(command, end_time_secs=time.time() + duration_s)

        except Exception as e:
            print(f"Error moving Spot robot: {e}")

    def turn_spot(self, target_angle_degrees, duration_s):
        try:
            command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            target_angle_radians = math.radians(target_angle_degrees)

            # Calculate angular velocity
            max_angular_velocity = 1.6  # radians per second
            angular_velocity = min(target_angle_radians / duration_s, max_angular_velocity)

            # Adjust duration if necessary
            actual_duration = target_angle_radians / angular_velocity

            command = RobotCommandBuilder.synchro_velocity_command(v_x=0.0, v_y=0.0, v_rot=angular_velocity)
            end_time = time.time() + actual_duration
            print(f"Sending turn command with v_rot={angular_velocity} radians/sec for duration_s={actual_duration} seconds (end_time={end_time})")

            # Use synchronous command for testing
            command_client.robot_command(command, end_time_secs=end_time)

            print("Turn command sent successfully")
        except Exception as e:
            print(f"Error turning Spot robot: {e}")

    def stop_spot(self):
        try:
            command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            command = RobotCommandBuilder.synchro_velocity_command(v_x=0.0, v_y=0.0, v_rot=0.0)
            command_client.robot_command(command)
        except Exception as e:
            print(f"Error stopping Spot robot: {e}")

def main():
    try:
        spot = SpotWrapper()
        spot.stand_spot()

        print("The command executed here")

        # Fetching GPS coordinates and calculating turn angle and distance
        DEST_LAT = 39.260150
        DEST_LON = -76.713600
        curr_lat = 39.260130
        curr_lon = -76.714656

        track = calculate_bearing(curr_lat, curr_lon, DEST_LAT, DEST_LON)
        print("Track: %s" % track)
        turn_angle = (track - 90) % 360  # Assuming a turn 90 degrees left from the current direction

        # Adjust turn angle to one decimal place
        rounded_turn_angle = round(turn_angle, 1)

        if rounded_turn_angle > 90:
            duration_s = int(rounded_turn_angle / 90)
        else:
            duration_s = 1  # Default duration if not exceeded 90 degrees

        print(f"Turning robot by {rounded_turn_angle} degrees")
        spot.turn_spot(rounded_turn_angle, duration_s)

        print("The turn command executed here")
        time.sleep(duration_s + 1)  # Ensure turn is completed before moving

        # Calculate distance to destination
        distance_to_dest = haversine(curr_lat, curr_lon, DEST_LAT, DEST_LON)
        print("Distance in meters: %s" % distance_to_dest)
        goal_x = distance_to_dest  # Assuming moving towards the destination in x direction
        goal_y = 0.0
        duration = 20
        print(f"Moving robot to position ({goal_x}, {goal_y}) for {duration} seconds")
        spot.move_spot(goal_x, goal_y, duration)
        return True
    except Exception as exc:
        print(f"An error occurred: {exc}")
        return False

if (__name__ == '__main__'):
    if not main():
        sys.exit(1)
