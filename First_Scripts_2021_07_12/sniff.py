#!/usr/bin/env python3

# Import the API module
import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand
from bosdyn.api import geometry_pb2 as geo
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, VISION_FRAME_NAME, get_odom_tform_body, get_vision_tform_body
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive

# Create an sdk object (the name is arbitrary)
sdk = bosdyn.client.create_standard_sdk('understanding-spot')

# Create a connection to the robot
robot = sdk.create_robot('192.168.50.3')

# Get the client ID
id_client = robot.ensure_client('robot-id')
spot_id = id_client.get_id()
print(f'Spot Id:\n{spot_id}')

# Log into the robot
robot.authenticate('student_HSD', 'dgHGcrD43SCgl')

# Get the robot state
state_client = robot.ensure_client('robot-state')
spot_state = state_client.get_robot_state()
print(f'Spot State:\n{spot_state}')

# Create an estop client and get the estop status
estop_client = robot.ensure_client('estop')
spot_estop_status = estop_client.get_status()
print(f'Spot estop status:\n{spot_estop_status}')

# Create an EStop end point
estop_endpoint = bosdyn.client.estop.EstopEndpoint(client=estop_client, name='my_estop', estop_timeout=9.0)
estop_endpoint.force_simple_setup()
print('Spot estopped')

# Spot will be estopped at this point

# To clear the estop, you must establish a keep-alive
estop_keep_alive = bosdyn.client.estop.EstopKeepAlive(estop_endpoint)
spot_estop_status = estop_client.get_status()
print(f'Spot estop status:\n{spot_estop_status}')

# List current leases
lease_client = robot.ensure_client('lease')
spot_lease_list = lease_client.list_leases()
print(f'Spot lease list:\n{spot_lease_list}')

# To obtain a lease
lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(lease_client)
lease = lease_client.acquire()
spot_lease_list = lease_client.list_leases()
print(f'Spot lease list:\n{spot_lease_list}')

# Powering Spot on
robot.power_on(timeout_sec=20)
spot_is_on = robot.is_powered_on()
print(f'Spot is powered { "up" if spot_is_on else "down" }')

# Establish timesync
robot.time_sync.wait_for_sync()

# Making Spot roll over
print('Spot rolling to the right in 5 seconds. PLEASE STAND CLEAR.')
command_client = robot.ensure_client(RobotCommandClient.default_service_name)
import time
time.sleep(5.0)

#   
standup = RobotCommandBuilder.robot_command.blocking_stand(command_client)
command_client.robot_command(standup)
time.sleep(7.0)

vision_tform_body = get_vision_tform_body(robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)
odom_tform_body = get_odom_tform_body(robot_state_client.get_robot_stat().kinematic_state.transforms_snapshot)

# EStop (cut_immediately=False will cause Spot to sit down before powering off
# cut_immediately=True will cause power to be cut immediately, and Spot will
# collapse)
robot.power_off(cut_immediately=False)
