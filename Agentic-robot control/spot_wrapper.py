import math
import numpy as np
import time
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, blocking_sit
from bosdyn.client.power import PowerClient
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive

class SpotWrapper:
    v_x=0.5 # Speed in meters per second in x direction
    v_y=0.0
    v_rot=0.0
    duration_s=1.0 # Distance in feet
    def __init__(self, spot_init=True):
        # Initialize Spot Robot if required
        if spot_init:
            self.sdk = bosdyn.client.create_standard_sdk('MoveSpot')
            self.robot = self.sdk.create_robot('robot_ip')
            # bosdyn.client.util.authenticate(self.robot)
            self.robot.authenticate('robot_user', 'robot_password')
            self.robot.time_sync.wait_for_sync()
            lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
            lease = lease_client.take()
            lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(lease_client)
            #with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=False):
            self.robot.power_on(timeout_sec=20)
            assert self.robot.is_powered_on(), "Robot failed to power on"

    def stand_spot(self):
        try:
            command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            blocking_stand(command_client, timeout_sec=15)
        except Exception as e:
            print(f"Error moving Spot robot: {e}")

    def sit_spot(self):
        try:
            command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            blocking_sit(command_client, timeout_sec=10)
        except Exception as e:
            print(f"Error sitting Spot robot: {e}")

    def move_spot(self,v_x,duration_s):
        try:
            if v_x > 1.5 and duration_s > 5:
              self.v_x = 2.0
              self.duration_s = 5.0
            else:  
              self.v_x = v_x
              self.duration_s = duration_s
            command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            command = RobotCommandBuilder.synchro_velocity_command(v_x=self.v_x, v_y=0.0, v_rot=0.0)
            command_client.robot_command(command, end_time_secs=time.time() + self.duration_s)
            time.sleep(self.duration_s)
            #time.sleep(5)  # Wait for movement to complete

        except Exception as e:
            print(f"Error moving Spot robot: {e}")

    def turn_spot(self,v_rot,duration_s):
        try:
            self.v_rot = v_rot
            self.duration_s = duration_s
            command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            command = RobotCommandBuilder.synchro_velocity_command(v_x=0.0, v_y=0.0, v_rot=self.v_rot)
            command_client.robot_command(command, end_time_secs=time.time() + self.duration_s)
            #time.sleep(5)  # Wait for movement to complete

        except Exception as e:
            print(f"Error moving Spot robot: {e}")

    def stop_spot(self):
        try:
            #self.v_rot = v_rot
            #self.duration_s = duration_s
            command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
            command = RobotCommandBuilder.synchro_velocity_command(v_x=0.5, v_y=0.0, v_rot=0)
            command_client.robot_command(command, end_time_secs=time.time() + 0.5)
            #time.sleep(5)  # Wait for movement to complete

        except Exception as e:
            print(f"Error moving Spot robot: {e}")
