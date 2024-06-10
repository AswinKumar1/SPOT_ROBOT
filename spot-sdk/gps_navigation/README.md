# Spot Robot GPS Navigation Script

This repository contains a Python script for controlling a Boston Dynamics Spot robot using the `bosdyn.client` library. The script demonstrates how to connect to the Spot robot, authenticate, retrieve GPS coordinates from a MongoDB database, and command the robot to move to a target location based on these coordinates. The script includes logging for better traceability and debugging.

## Requirements

- Python 3.6+
- `bosdyn-client` Python package
- `pymongo` Python package
- `geopy` Python package
- A Boston Dynamics Spot robot with network access and proper authentication credentials
- A MongoDB server with the necessary GPS data

## Installation

1. **Clone the repository:**
   ```sh
   git clone https://github.com/yourusername/spot_gps_navigation.git
   cd spot_gps_navigation
   ```

2. **Install the necessary Python packages:**
   ```sh
   pip install bosdyn-client pymongo geopy
   ```

## Usage

1. **Update the script with your Spot robot's IP address and authentication credentials:**

   ```python
   robot = sdk.create_robot(options.hostname)
   robot.authenticate('admin', 'Cards@2023spot')
   ```

2. **Update the script with your MongoDB connection details:**

   ```python
   db_hostname = '192.168.1x.xx'
   db_port = 27017
   ```

3. **Run the script:**
   ```sh
   python spot_gps_navigation.py --hostname 192.168.x.xx
   ```

   Use the `--frame` option to specify the frame (`vision` or `odom`), and the `--stairs` option if you want the robot to move in stairs mode.

   Example:
   ```sh
   python spot_gps_navigation.py --hostname 192.168.1.12 --frame odom --stairs
   ```

## Script Explanation

### Imports

The script imports various modules from the `bosdyn.client` and `bosdyn.api` libraries to interact with the Spot robot and send commands. It also imports the `logging`, `math`, `sys`, `time`, `pymongo`, and `geopy` libraries for logging, mathematical calculations, system operations, time management, MongoDB operations, and geospatial calculations, respectively.

### Logging Setup

The script sets up logging to provide information and error messages:
```python
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
```

### Main Function

The main function performs the following steps:

1. **Argument Parsing:**
   ```python
   import argparse
   parser = argparse.ArgumentParser()
   bosdyn.client.util.add_base_arguments(parser)
   parser.add_argument('hostname', help='Hostname or IP address of the robot.')
   parser.add_argument('--frame', choices=[VISION_FRAME_NAME, ODOM_FRAME_NAME],
                       default=ODOM_FRAME_NAME, help='Send the command in this frame.')
   parser.add_argument('--stairs', action='store_true', help='Move the robot in stairs mode.')
   options = parser.parse_args()
   ```

2. **Create and Authenticate Robot Instance:**
   ```python
   sdk = bosdyn.client.create_standard_sdk('SpotGPSClient')
   robot = sdk.create_robot(options.hostname)
   robot.authenticate('xxxxx', 'xxxxxxxxxxx')
   ```

3. **Check Estop:**
   ```python
   assert not robot.is_estopped(), 'Robot is estopped. Please use an external E-Stop client.'
   ```

4. **Setup Lease, State, and Command Clients:**
   ```python
   lease_client = robot.ensure_client(LeaseClient.default_service_name)
   robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
   robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
   ```

5. **Acquire Lease and Power On Robot:**
   ```python
   with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
       robot.time_sync.wait_for_sync()
       robot.power_on()
       blocking_stand(robot_command_client)
   ```

6. **Retrieve GPS Locations from MongoDB:**
   ```python
   current_latitude, current_longitude, target_latitude, target_longitude = get_gps_locations_from_db()
   ```

7. **Convert GPS to Local Frame:**
   ```python
   dx, dy = gps_to_local_frame(current_latitude, current_longitude, target_latitude, target_longitude)
   ```

8. **Move Robot to Target Location:**
   ```python
   return relative_move(dx, dy, 0, options.frame, robot_command_client, robot_state_client, stairs=options.stairs)
   ```

### Helper Functions

1. **get_gps_locations_from_db:**
   Connects to MongoDB, retrieves the current and target GPS locations for the robot, and returns them.

2. **gps_to_local_frame:**
   Converts GPS coordinates to local frame coordinates (dx, dy) in meters.

3. **relative_move:**
   Commands the robot to move to the target location in the specified frame and waits until the robot reaches the goal.
