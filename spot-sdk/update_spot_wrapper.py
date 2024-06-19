import sys
import time
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, blocking_sit
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
import logging
import os
from bosdyn.api import geometry_pb2  # Importing necessary protobuf module

# logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SpotWrapper:
    def __init__(self, spot_init=True):
        self.sdk = None
        self.robot = None
        self.lease = None
        self.lease_keep_alive = None

        if spot_init:
            try:
                # Initialize Spot Robot if required
                self.sdk = bosdyn.client.create_standard_sdk('MoveSpot')
                robot_ip = os.getenv('SPOT_IP')
                self.robot = self.sdk.create_robot(robot_ip)
                username = os.getenv('SPOT_USERNAME')
                password = os.getenv('SPOT_PASSWORD')
                self.robot.authenticate(username, password)
                self.robot.time_sync.wait_for_sync()
                lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
                self.lease = lease_client.take()
                self.lease_keep_alive = LeaseKeepAlive(lease_client)
                self.robot.power_on(timeout_sec=20)
                assert self.robot.is_powered_on(), "Robot failed to power on"
            except Exception as e:
                logger.error(f"Initialization error: {e}", exc_info=True)

    def __del__(self):
        if self.lease:
            try:
                lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
                lease_client.return_lease(self.lease)
            except Exception as e:
                logger.error(f"Error returning lease: {e}", exc_info=True)

    def stand_spot(self):
        try:
            command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            blocking_stand(command_client, timeout_sec=15)
        except Exception as e:
            logger.error(f"Error standing Spot robot: {e}", exc_info=True)

    def sit_spot(self):
        try:
            command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            blocking_sit(command_client, timeout_sec=10)
        except Exception as e:
            logger.error(f"Error sitting Spot robot: {e}", exc_info=True)

    def move_spot(self, goal_x, duration_s):
        try:
            command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            frame_tree_snapshot = self.robot.get_frame_tree_snapshot()

            # Define mobility parameters for the move command
            obstacle_params = spot_command_pb2.ObstacleParams(
                disable_vision_foot_obstacle_avoidance=False,
                disable_vision_body_obstacle_avoidance=False
            )
            body_control_params = spot_command_pb2.BodyControlParams()
            max_linear_velocity = geometry_pb2.Vec2(x=0.5, y=0.0)
            max_angular_velocity = 0.0
            max_se2_velocity = geometry_pb2.SE2Velocity(linear=max_linear_velocity, angular=max_angular_velocity)
            velocity_limit = geometry_pb2.SE2VelocityLimit(max_vel=max_se2_velocity)

            mobility_params = spot_command_pb2.MobilityParams(
                body_control=body_control_params,
                obstacle_params=obstacle_params,
                locomotion_hint=spot_command_pb2.HINT_AUTO,
                vel_limit=velocity_limit
            )

            command = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(
                goal_x_rt_body=goal_x,
                goal_y_rt_body=0.0,
                goal_heading_rt_body=0.0,
                frame_tree_snapshot=frame_tree_snapshot,
                params=mobility_params
            )
            command_client.robot_command(command, end_time_secs=time.time() + duration_s)
        except Exception as e:
            logger.error(f"Error moving Spot robot: {e}", exc_info=True)

    def proceed_spot(self, goal_x, duration_s):
        try:
            command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            frame_tree_snapshot = self.robot.get_frame_tree_snapshot()

            # Define mobility parameters for the move command
            obstacle_params = spot_command_pb2.ObstacleParams(
                disable_vision_foot_obstacle_avoidance=True,
                disable_vision_body_obstacle_avoidance=True
            )
            body_control_params = spot_command_pb2.BodyControlParams()
            max_linear_velocity = geometry_pb2.Vec2(x=0.5, y=0.0)
            max_angular_velocity = 0.0
            max_se2_velocity = geometry_pb2.SE2Velocity(linear=max_linear_velocity, angular=max_angular_velocity)
            velocity_limit = geometry_pb2.SE2VelocityLimit(max_vel=max_se2_velocity)

            mobility_params = spot_command_pb2.MobilityParams(
                body_control=body_control_params,
                obstacle_params=obstacle_params,
                locomotion_hint=spot_command_pb2.HINT_AUTO,
                vel_limit=velocity_limit
            )

            command = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(
                goal_x_rt_body=goal_x,
                goal_y_rt_body=0.0,
                goal_heading_rt_body=0.0,
                frame_tree_snapshot=frame_tree_snapshot,
                params=mobility_params
            )
            command_client.robot_command(command, end_time_secs=time.time() + duration_s)
        except Exception as e:
            logger.error(f"Error moving Spot robot: {e}", exc_info=True)

    def turn_spot(self, goal_heading, duration_s):
        try:
            command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            frame_tree_snapshot = self.robot.get_frame_tree_snapshot()

            # Define mobility parameters for the turn command
            obstacle_params = spot_command_pb2.ObstacleParams(
                disable_vision_foot_obstacle_avoidance=False,
                disable_vision_body_obstacle_avoidance=False
            )
            body_control_params = spot_command_pb2.BodyControlParams()
            max_linear_velocity = geometry_pb2.Vec2(x=0.0, y=0.0)
            max_angular_velocity = 0.5  # Assuming 0.5 rad/s as an example

            max_se2_velocity = geometry_pb2.SE2Velocity(linear=max_linear_velocity, angular=max_angular_velocity)
            velocity_limit = geometry_pb2.SE2VelocityLimit(max_vel=max_se2_velocity)

            mobility_params = spot_command_pb2.MobilityParams(
                body_control=body_control_params,
                obstacle_params=obstacle_params,
                locomotion_hint=spot_command_pb2.HINT_AUTO,
                vel_limit=velocity_limit
            )

            command = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(
                goal_x_rt_body=0.0,
                goal_y_rt_body=0.0,
                goal_heading_rt_body=goal_heading,
                frame_tree_snapshot=frame_tree_snapshot,
                params=mobility_params
            )
            command_client.robot_command_async(command, end_time_secs=time.time() + duration_s)
        except Exception as e:
            logger.error(f"Error turning Spot robot: {e}", exc_info=True)

    def stop_spot(self):
        try:
            command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            frame_tree_snapshot = self.robot.get_frame_tree_snapshot()

            # Define mobility parameters for the stop command
            obstacle_params = spot_command_pb2.ObstacleParams(
                disable_vision_foot_obstacle_avoidance=False,
                disable_vision_body_obstacle_avoidance=False
            )
            body_control_params = spot_command_pb2.BodyControlParams()
            max_linear_velocity = geometry_pb2.Vec2(x=0.0, y=0.0)
            max_angular_velocity = 0.0

            max_se2_velocity = geometry_pb2.SE2Velocity(linear=max_linear_velocity, angular=max_angular_velocity)
            velocity_limit = geometry_pb2.SE2VelocityLimit(max_vel=max_se2_velocity)

            mobility_params = spot_command_pb2.MobilityParams(
                body_control=body_control_params,
                obstacle_params=obstacle_params,
                locomotion_hint=spot_command_pb2.HINT_AUTO,
                vel_limit=velocity_limit
            )

            command = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(
                goal_x_rt_body=0.0,
                goal_y_rt_body=0.0,
                goal_heading_rt_body=0.0,
                frame_tree_snapshot=frame_tree_snapshot,
                params=mobility_params
            )
            command_client.robot_command(command)
        except Exception as e:
            logger.error(f"Error stopping Spot robot: {e}", exc_info=True)

def main():
    try:
        spot = SpotWrapper()
        spot.stand_spot()

        logger.info("The command executed here")

        goal_x = 5.0
        duration_s = 20.0
        spot.move_spot(goal_x, duration_s)

        return True
    except Exception as exc:
        logger.error(f"An error occurred: {exc}", exc_info=True)
        return False

if __name__ == '__main__':
    if not main():
        sys.exit(1)
