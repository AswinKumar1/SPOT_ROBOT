# Spot Robot Command Script

This repository contains a Python script for controlling a Boston Dynamics Spot robot using the `bosdyn.client` library. The script demonstrates how to connect to the Spot robot, authenticate, and send a mobility command with specific parameters, including obstacle avoidance and velocity limits. It also includes logging for better traceability and debugging.

## Requirements

- Python 3.6+
- `bosdyn-client` Python package
- A Boston Dynamics Spot robot with network access and proper authentication credentials

## Installation

1. **Clone the repository:**
   ```sh
   git clone https://github.com/yourusername/spot_robot_command.git
   cd spot_robot_command
   ```

2. **Install the necessary Python packages:**
   ```sh
   pip install bosdyn-client
   ```

## Usage

1. **Update the script with your Spot robot's IP address and authentication credentials:**

   ```python
   robot = sdk.create_robot('192.168.1.12')
   robot.authenticate('admin', 'Cards@2023spot')
   ```

2. **Run the script:**
   ```sh
   python spot_robot_command.py
   ```

## Script Explanation

### Imports

The script imports various modules from the `bosdyn.client` and `bosdyn.api` libraries to interact with the Spot robot and send commands. It also imports the `logging` module for logging purposes.

### Logging Setup

The script sets up logging to provide information and error messages:
```python
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
```

### Main Function

The main function performs the following steps:

1. **Initialize the SDK and create a robot instance:**
   ```python
   sdk = bosdyn.client.create_standard_sdk('SpotClient')
   robot = sdk.create_robot('192.168.1.12')
   robot.authenticate('admin', 'Cards@2023spot')
   robot.time_sync.wait_for_sync()
   ```

2. **Acquire a lease and keep it alive:**
   ```python
   lease_client = robot.ensure_client(LeaseClient.default_service_name)
   lease = lease_client.take()
   lease_keep_alive = LeaseKeepAlive(lease_client)
   ```

3. **Create a RobotCommandClient:**
   ```python
   robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
   ```

4. **Define obstacle parameters to disable vision-based obstacle avoidance:**
   ```python
   obstacle_params = robot_command_pb2.ObstacleParams(
       disable_vision_foot_obstacle_avoidance=True,
       disable_vision_body_obstacle_avoidance=True
   )
   ```

5. **Define body control parameters:**
   ```python
   body_control_params = robot_command_pb2.BodyControlParams()
   ```

6. **Set velocity limits:**
   ```python
   max_linear_velocity = geometry_pb2.Vec2(x=0.5, y=0.0)
   max_angular_velocity = 0.0
   max_se2_velocity = geometry_pb2.SE2Velocity(linear=max_linear_velocity, angular=max_angular_velocity)
   velocity_limit = geometry_pb2.SE2VelocityLimit(max_vel=max_se2_velocity)
   ```

7. **Define mobility parameters with velocity limits:**
   ```python
   mobility_params = robot_command_pb2.MobilityParams(
       body_control=body_control_params,
       obstacle_params=obstacle_params,
       locomotion_hint=robot_command_pb2.HINT_AUTO,
       vel_limit=velocity_limit
   )
   ```

8. **Get the frame tree snapshot:**
   ```python
   frame_tree_snapshot = robot.get_frame_tree_snapshot()
   ```

9. **Create the RobotCommand:**
   ```python
   command = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(
       goal_x_rt_body=2.0,
       goal_y_rt_body=0.0,
       goal_heading_rt_body=0.0,
       frame_tree_snapshot=frame_tree_snapshot,
       params=mobility_params
   )
   ```

10. **Send the command to the robot and log the success message:**
    ```python
    robot_command_client.robot_command(command, end_time_secs=time.time() + 5)
    logger.info("Command sent successfully")
    ```

### Exception Handling

The script includes basic exception handling to log any errors that occur during execution:
```python
except Exception as e:
    logger.error(f"Failed to execute command: {e}")
```
