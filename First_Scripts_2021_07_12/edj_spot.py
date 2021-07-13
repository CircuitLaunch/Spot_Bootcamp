#!/usr/bin/env python3

# Import the API module
import argparse
import sys
import numpy as np
import math
import traceback
import time

from bosdyn.api import robot_command_pb2, synchronized_command_pb2, mobility_command_pb2, basic_command_pb2, geometry_pb2, trajectory_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME, get_vision_tform_body
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.util import seconds_to_duration

# User-set params
# duration of the whole move [s]
_SECONDS_FULL = 15
# length of the square the robot walks [m]
_L_ROBOT_SQUARE = 0.5


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

# Making Spot stand
print('Spot standing in 5 seconds. PLEASE STAND CLEAR.')
import time
time.sleep(5.0)
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand
command_client = robot.ensure_client(RobotCommandClient.default_service_name)
blocking_stand(command_client, timeout_sec=10)

robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)

robot_state = robot_state_client.get_robot_state()
vision_T_world = get_vision_tform_body(robot_state.kinematic_state.transforms_snapshot)

# In this demo, the robot will walk in a square.
# There are some parameters that you can set below:

# Initialize a robot command message, which we will build out below
command = robot_command_pb2.RobotCommand()

# points in the square
x_vals = np.array([0, 1, 1, 0, 0])
y_vals = np.array([0, 0, 1, 1, 0])

# duration in seconds for each move
seconds_body = _SECONDS_FULL / x_vals.size

# Commands will be sent in the visual odometry ("vision") frame
frame_name = VISION_FRAME_NAME

# Build a body se2trajectory by first assembling points
for ii in range(x_vals.size):
    # Pull the point in the square relative to the robot and scale according to param
    x = _L_ROBOT_SQUARE * x_vals[ii]
    y = _L_ROBOT_SQUARE * y_vals[ii]

    # Transform desired position into world frame
    x_ewrt_vision, y_ewrt_vision, z_ewrt_vision = vision_T_world.transform_point(x, y, 0)

    # Add a new point to the robot command's arm cartesian command se3 trajectory
    # This will be an se2 trajectory point
    point = command.synchronized_command.mobility_command.se2_trajectory_request.trajectory.points.add()

    # Populate this point with the desired position, angle, and duration information
    point.pose.position.x = x_ewrt_vision
    point.pose.position.y = y_ewrt_vision

    point.pose.angle = vision_T_world.rot.to_yaw()

    traj_time = (ii + 1) * seconds_body
    duration = seconds_to_duration(traj_time)
    point.time_since_reference.CopyFrom(duration)

# set the frame for the body trajectory
command.synchronized_command.mobility_command.se2_trajectory_request.se2_frame_name = frame_name

# Constrain the robot not to turn, forcing it to strafe laterally.
speed_limit = SE2VelocityLimit(max_vel=SE2Velocity(linear=Vec2(x=2, y=2), angular=0),
                               min_vel=SE2Velocity(linear=Vec2(x=-2, y=-2), angular=0))
mobility_params = spot_command_pb2.MobilityParams(vel_limit=speed_limit)

command.synchronized_command.mobility_command.params.CopyFrom(RobotCommandBuilder._to_any(mobility_params))

# Send the command using the command client
# The SE2TrajectoryRequest requires an end_time, which is set
# during the command client call
robot.logger.info("Sending body trajectory commands.")
command_client.robot_command(command, end_time_secs=time.time() + _SECONDS_FULL)
time.sleep(_SECONDS_FULL + 2)

# Power the robot off. By specifying "cut_immediately=False", a safe power off command
# is issued to the robot. This will attempt to sit the robot before powering off.
robot.power_off(cut_immediately=False, timeout_sec=20)
assert not robot.is_powered_on(), "Robot power off failed."
robot.logger.info("Robot safely powered off.")

# If we successfully acquired a lease, return it.
lease_client.return_lease(lease)
