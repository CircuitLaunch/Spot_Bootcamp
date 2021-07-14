#!/usr/bin/env python3

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

# Stand up Spot!
print('Spot Standing up 5 seconds. PLEASE STAND CLEAR.')
command_client = robot.ensure_client(RobotCommandClient.default_service_name)
import time
time.sleep(5.0)

#   
blocking_stand(command_client,timeout_sec=10)
time.sleep(3.0)

#Get a snapshot of the current robot platform state
#vision_tform_body = get_vision_tform_body(robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)
#odom_tform_body = get_odom_tform_body(robot_state_client.get_robot_stat().kinematic_state.transforms_snapshot)

#body_transform_goal = math_helpers.SE3Pose(x=1,y=0,z=0,rot=math_helpers.Quat())

#Multiply current vision transform by goal transform
#vision_tform_goal = vision_tform_body * body_tform_goal 

#robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(goal_x=vision_tform_goal.x,
#                                                     goal_y=vision_tform_goal.y,
#                                                     goal_heading=vision_tform_goal.rot.to_yaw(),
#                                                     frame_name=VISION_FRAME_NAME)
#end_time=2.0 
#robot_command_client.robot_command(lease=None, command=robot_cmd, end_time_secs=time.time() + end_time)

# Get new robot state information after moving the robot. Here we are getting the transform odom_tform_body,
    # which describes the robot body's position in the odom frame.
#odom_tform_body = get_odom_tform_body( robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)

#time.sleep(1.0)
#Pitch robot forward to make it 'sniff'
#"Sniffing" is combination of pitch forward
#turn left/pause/turn right/pause center/pause, move_forward, repeat

footprint_R_body = EulerZXY(yaw=0.0, roll=0.0, pitch=1.31)
cmd = RobotCommandBuilder.stand_command(footprint_R_body=footprint_R_body)
command_client.robot_command(cmd)
time.sleep(2.0)

footprint_R_body = EulerZXY(yaw=0.5, roll=0.0, pitch=1.31)
cmd = RobotCommandBuilder.stand_command(footprint_R_body=footprint_R_body)
command_client.robot_command(cmd)
time.sleep(2.0)

footprint_R_body = EulerZXY(yaw=-0.5, roll=0.0, pitch=1.31)
cmd = RobotCommandBuilder.stand_command(footprint_R_body=footprint_R_body)
command_client.robot_command(cmd)
time.sleep(5.0)

footprint_R_body = EulerZXY(yaw=0.0, roll=0.5, pitch=0.0)
cmd = RobotCommandBuilder.stand_command(footprint_R_body=footprint_R_body)
command_client.robot_command(cmd)
time.sleep(0.2)

footprint_R_body = EulerZXY(yaw=0.0,roll=-0.25, pitch=0.0)
cmd = RobotCommandBuilder.stand_command(footprint_R_body=footprint_R_body)
command_client.robot_command(cmd)
time.sleep(0.2)

footprint_R_body = EulerZXY(yaw=0.0, roll=0.5, pitch=0.0)
cmd = RobotCommandBuilder.stand_command(footprint_R_body=footprint_R_body)
command_client.robot_command(cmd)
time.sleep(0.2)

footprint_R_body = EulerZXY(yaw=0.0, roll=-0.5, pitch=0.0)
cmd = RobotCommandBuilder.stand_command(footprint_R_body=footprint_R_body)
command_client.robot_command(cmd)
time.sleep(0.2)

footprint_R_body = EulerZXY(yaw=0.0, roll=0.5, pitch=0.0)
cmd = RobotCommandBuilder.stand_command(footprint_R_body=footprint_R_body)
command_client.robot_command(cmd)
time.sleep(0.2)

footprint_R_body = EulerZXY(yaw=0.0, roll=-0.5, pitch=0.0)
cmd = RobotCommandBuilder.stand_command(footprint_R_body=footprint_R_body)
command_client.robot_command(cmd)
time.sleep(0.2)

footprint_R_body = EulerZXY(yaw=0.0, roll=0.5, pitch=0.0)
cmd = RobotCommandBuilder.stand_command(footprint_R_body=footprint_R_body)
command_client.robot_command(cmd)
time.sleep(1.0)

footprint_R_body = EulerZXY(yaw=0.0, roll=0.0, pitch=0.0)
cmd = RobotCommandBuilder.stand_command(footprint_R_body=footprint_R_body)
command_client.robot_command(cmd)
time.sleep(5.0)

# EStop (cut_immediately=False will cause Spot to sit down before powering off
# cut_immediately=True will cause power to be cut immediately, and Spot will
# collapse)
robot.power_off(cut_immediately=False)
