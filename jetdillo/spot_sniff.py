#!/usr/bin/env python3

#Python imports
import time
# Import the API module
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand
from bosdyn.api import geometry_pb2 as geo
from bosdyn.client import math_helpers
from bosdyn.geometry import EulerZXY
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, VISION_FRAME_NAME, get_odom_tform_body, get_vision_tform_body
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive

class Doggo:

   def __init__(self):
#
# Create an sdk object (the name is arbitrary)
      self.sdk = bosdyn.client.create_standard_sdk('understanding-spot')

# Create a connection to the robot
      self.robot = sdk.create_robot('192.168.50.3')

# Get the client ID
      self.id_client = robot.ensure_client('robot-id')
      self.spot_id = id_client.get_id()
      print(f'Spot Id:\n{spot_id}')

# Log into the robot
      self.robot.authenticate('student_HSD', 'dgHGcrD43SCgl')

# Get the robot state
      self.state_client = robot.ensure_client('robot-state')
      self.spot_state = self.state_client.get_robot_state()
      print(f'Spot State:\n{spot_state}')

# Create an estop client and get the estop status
      self.estop_client = self.robot.ensure_client('estop')
      self.spot_estop_status = self.estop_client.get_status()
      print(f'Spot estop status:\n{spot_estop_status}')

# Create an EStop end point
      self.estop_endpoint = bosdyn.client.estop.EstopEndpoint(client=estop_client, name='my_estop', estop_timeout=9.0)
      self.estop_endpoint.force_simple_setup()
      print('Spot estopped')

# Spot will be estopped at this point

# To clear the estop, you must establish a keep-alive
      self.estop_keep_alive = bosdyn.client.estop.EstopKeepAlive(estop_endpoint)
      self.spot_estop_status = estop_client.get_status()
      print(f'Spot estop status:\n{spot_estop_status}')

# To obtain a lease
      self.lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(lease_client)
      self.lease = lease_client.acquire()
      self.spot_lease_list = lease_client.list_leases()
      print(f'Spot lease list:\n{spot_lease_list}')

# Powering Spot on
      self.robot.power_on(timeout_sec=20)
      spot_is_on = self.robot.is_powered_on()
      print(f'Spot is powered { "up" if spot_is_on else "down" }')

# Establish timesync
      self.robot.time_sync.wait_for_sync()

#Clear robot Euler Pose
      footprint_R_body = EulerZXY(yaw=0.0, roll=0.0, pitch=0)

# Stand up Spot!
   def stand(self):
      print('Spot Standing up 5 seconds. PLEASE STAND CLEAR.')
      self.command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
      time.sleep(5.0)
      blocking_stand(command_client,timeout_sec=10)

   def sniff(self,r,p,y,pose_delay=2.0):
      footprint_R_body = EulerZXY(yaw=y, roll=r, pitch=p)
      cmd = RobotCommandBuilder.stand_command(footprint_R_body=footprint_R_body)
      command_client.robot_command(cmd)
      time.sleep(pose_delay)

   def shake(self,roll_left,roll_right,shake_count=5,shake_delay=0.25):
      
      for c in range(0,shake_count):
         footprint_R_body = EulerZXY(yaw=0, roll=roll_left, pitch=0)
         cmd = RobotCommandBuilder.stand_command(footprint_R_body=footprint_R_body)
         command_client.robot_command(cmd)
         time.sleep(shake_delay)
         footprint_R_body = EulerZXY(yaw=0, roll=roll_right, pitch=0)
         cmd = RobotCommandBuilder.stand_command(footprint_R_body=footprint_R_body)
         command_client.robot_command(cmd)
    
   def zero_stand(self):
      footprint_R_body = EulerZXY(yaw=0.0, roll=0.0, pitch=0.0)
      cmd = RobotCommandBuilder.stand_command(footprint_R_body=footprint_R_body)
      command_client.robot_command(cmd)

   def done(self):
      robot.power_off(cut_immediately=False)

if __name__ == '__main__':
   spot = Doggo()

   spot.sniff(r=0.0,p=0.5,y=0.0,delay=1.0)
   spot.sniff(r=0.0,p=0.5,y=0.5,delay=2.0)
   spot.sniff(r=0.0,p=0.5,y=-0.5,delay=2.0)

   time.sleep(5)
   spot.zero_stand()
   spot.shake(0.5,-0.5,shake_count=10,shake_delay=0.25)
