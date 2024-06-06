import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.api import geometry_pb2, basic_command_pb2
from bosdyn.api.spot import robot_command_pb2
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
import time
import logging

# For logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

if __name__ == "__main__":
    try:
        sdk = bosdyn.client.create_standard_sdk('SpotClient')
        robot = sdk.create_robot('192.168.1.12')
        robot.authenticate('admin', 'Cards@2023spot')
        robot.time_sync.wait_for_sync()

        lease_client = robot.ensure_client(LeaseClient.default_service_name)
        lease = lease_client.take()
        lease_keep_alive = LeaseKeepAlive(lease_client)
        robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)

        # Define obstacle parameters
        obstacle_params = robot_command_pb2.ObstacleParams(
            disable_vision_foot_obstacle_avoidance=True,
            disable_vision_body_obstacle_avoidance=True
        )

        # Define body control parameters
        body_control_params = robot_command_pb2.BodyControlParams()
        
        # Define velocity limits
        max_linear_velocity = geometry_pb2.Vec2(x=0.5, y=0.0) 
        max_angular_velocity = 0.0  
        
        max_se2_velocity = geometry_pb2.SE2Velocity(linear=max_linear_velocity, angular=max_angular_velocity)

        velocity_limit = geometry_pb2.SE2VelocityLimit(
            max_vel=max_se2_velocity
        )

        # Define mobility parameters with velocity limits
        mobility_params = robot_command_pb2.MobilityParams(
            body_control=body_control_params,
            obstacle_params=obstacle_params,
            locomotion_hint=robot_command_pb2.HINT_AUTO,
            vel_limit=velocity_limit
        )

        # Get the frame tree snapshot
        frame_tree_snapshot = robot.get_frame_tree_snapshot()

        # Create the RobotCommand
        command = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(
            goal_x_rt_body=2.0,
            goal_y_rt_body=0.0,
            goal_heading_rt_body=0.0,
            frame_tree_snapshot=frame_tree_snapshot,
            params=mobility_params
        )

        # Send the command
        robot_command_client.robot_command(command, end_time_secs=time.time() + 5)
        logger.info("Command sent successfully")

    except Exception as e:
        logger.error(f"Failed to execute command: {e}")