import requests
import folium
import polyline
from bosdyn.client import create_standard_sdk
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.lease import LeaseClient
from bosdyn.client import ResponseError, RpcError
from bosdyn.api.geometry_pb2 import SE2Pose
from bosdyn.api import geometry_pb2, trajectory_pb2

def get_route(api_key, coordinates):
    url = "https://api.openrouteservice.org/v2/directions/driving-car"
    headers = {
        'Authorization': api_key,
        'Content-Type': 'application/json'
    }
    body = {
        'coordinates': coordinates,
        'instructions': False,
        'geometry_simplify': True,
    }
    
    response = requests.post(url, headers=headers, json=body)
    
    if response.status_code != 200:
        print(f"Error: Received status code {response.status_code}")
        return None
    
    try:
        data = response.json()
    except requests.exceptions.JSONDecodeError as e:
        print(f"JSON decode error: {e}")
        return None
    
    if 'routes' in data:
        route_polyline = data['routes'][0]['geometry']
        route_coordinates = polyline.decode(route_polyline)
        return route_coordinates
    else:
        print("No routes found in response")
        return None

def plot_route(route_coordinates, map_obj):
    folium.PolyLine(route_coordinates, color="blue", weight=5, opacity=0.7).add_to(map_obj)
    folium.Marker(route_coordinates[0], tooltip='Start', icon=folium.Icon(color='green')).add_to(map_obj)
    folium.Marker(route_coordinates[-1], tooltip='End', icon=folium.Icon(color='red')).add_to(map_obj)
    return map_obj

def input_coordinates():
    print("Enter start and end GPS coordinates.")
    start_lat = float(input("Start Latitude: "))
    start_lon = float(input("Start Longitude: "))
    end_lat = float(input("End Latitude: "))
    end_lon = float(input("End Longitude: "))
    return [[start_lon, start_lat], [end_lon, end_lat]]

def send_waypoints_to_spot(waypoints):
    sdk = create_standard_sdk('SpotClient')
    robot = sdk.create_robot('192.168.80.3')  # Replace with your robot's IP address
    robot.authenticate('user', 'password')   # Replace with your robot's username and password
    
    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    lease = lease_client.take()

    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    
    # Initialize the SE2 trajectory
    traj_points = []
    for lat, lon in waypoints:
        traj_points.append(trajectory_pb2.SE2TrajectoryPoint(
            pose=SE2Pose(position=geometry_pb2.Vec2(x=lat, y=lon), angle=0.0)))  

    traj = trajectory_pb2.SE2Trajectory(points=traj_points)

    end_time = robot.time_sync.endpoint.clock.get_robot_time_in_robot_reference().add(seconds=60)  
    frame_name = 'odom'  

    command = RobotCommandBuilder.synchro_se2_trajectory_command(
        traj, frame_name, end_time)

    try:
        command_client.robot_command(command)
    except (ResponseError, RpcError) as e:
        print(f"Failed to send command: {e}")
        lease_client.return_lease(lease)
        return

    lease_client.return_lease(lease)

# API Key
API_KEY = 'api_key'

# Request user to input coordinates
coordinates = input_coordinates()

# Get the route
route_coordinates = get_route(API_KEY, coordinates)

if route_coordinates:
    # Initialize a map centered around the first point
    map_obj = folium.Map(location=route_coordinates[0], zoom_start=14)
    
    # Plot the route on the map
    map_obj = plot_route(route_coordinates, map_obj)
    
    # Save and display the map
    map_obj.save('openroute_map.html')
    print("Map has been saved to 'openroute_map.html'")
    
    # Send waypoints to Spot robot
    send_waypoints_to_spot(route_coordinates)
else:
    print("Failed to retrieve or decode the route.")
