import bosdyn.client
import bosdyn.client.util
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand, RobotCommandBuilder
import time
import sys
import math
import argparse

from bosdyn.api import geometry_pb2 as geo
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, VISION_FRAME_NAME, get_odom_tform_body, get_vision_tform_body


# Trying to import stuff for proper end time calculations
from bosdyn.api import power_pb2


## --- Function Declarations --- 

# Delay Function -- to make things slightly easier
def delay():
    time.sleep(end_time_add + sleep_adder)

    return



## --- Setup and object instance declarations ---

sdk = bosdyn.client.create_standard_sdk('understanding-spot')
robot = sdk.create_robot('192.168.50.3')
id_client = robot.ensure_client('robot-id')
robot.authenticate('student_HSD', 'dgHGcrD43SCgl')
state_client = robot.ensure_client('robot-state')

# --- E-stop ---
estop_client = robot.ensure_client('estop')

estop_endpoint = bosdyn.client.estop.EstopEndpoint(client=estop_client, name='my_estop', estop_timeout=9.0)

estop_endpoint.force_simple_setup()

estop_keep_alive = bosdyn.client.estop.EstopKeepAlive(estop_endpoint)
# --- End of E-stop ---

lease_client = robot.ensure_client('lease')
lease = lease_client.acquire()
lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(lease_client)

robot.power_on(timeout_sec=20)
print(f'Spot power: {robot.is_powered_on()}')

robot.time_sync.wait_for_sync()

## --- End of setup ---

# Setting up your command_client instances
robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)




## --- Command List ---

# -- End Time Declaration --
end_time_add = 10
sleep_adder = -5



# -- Original State Returner -- 

# - getting current state - 
vision_tform_body = get_vision_tform_body(
                    robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)
# - Creating ultimate state -
ultimate_tform_body = vision_tform_body

# - Moving spot - 
# RobotCommandBuilder instances
robot_vel_cmd = RobotCommandBuilder.velocity_command(0.5, 0, 0.5)

# Robot client executing command instances
robot_command_client.robot_command(robot_vel_cmd, end_time_secs=time.time() + end_time_add - 5)

# Delay
time.sleep(end_time_add + sleep_adder)

# - End of moving spot - 

# Getting current state again after movement
vision_tform_body_new = get_vision_tform_body(
                    robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)

# - While loop to bring Spot back to ultimate state - 
while vision_tform_body_new != ultimate_tform_body:
    # Get current state information
    vision_tform_body_new = get_vision_tform_body(
                    robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)
    # Setting target transform to ultimate state
    body_tform_goal_new = ultimate_tform_body
    #body_tform_goal = math_helpers.SE3Pose(x=-2, y=0, z=0, rot=math_helpers.Quat())
    # Crafting target trajectory by applying target transform to the current state
    vision_tform_goal_new = vision_tform_body_new * body_tform_goal_new
    # Create command instance
    robot_move_to_cmd_new = RobotCommandBuilder.synchro_se2_trajectory_point_command(
                            goal_x=vision_tform_goal_new.x,
                            goal_y=vision_tform_goal_new.y,
                            goal_heading=vision_tform_goal_new.rot.to_yaw(),
                            frame_name=VISION_FRAME_NAME)
    # Run the command instance using your robot_command_client
    robot_command_client.robot_command(robot_move_to_cmd_new, end_time_secs=time.time() + end_time_add)    
    
    # Delay between movements
    time.sleep(end_time_add + sleep_adder)

    # While loop end



# -- End of Original State Returner -- 

'''
# -- Vision transform movement command -- 

# -- Getting robot state info --
# Get the vision_tform_body transform to understand the robot's current position in the vision frame
vision_tform_body = get_vision_tform_body(
                    robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)

# -- Setting body goal --
body_tform_goal = math_helpers.SE3Pose(x=-2, y=0, z=0, rot=math_helpers.Quat())

# -- Setting goal for vision transform --
# Creates goal state for the vision transform by multiplying the current state (vision body) by the goal
# (body goal)
vision_tform_goal = vision_tform_body * body_tform_goal

# Move to location specified by the body and vision goal transforms
# Create command instance
robot_move_to_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(goal_x=vision_tform_goal.x,
                        goal_y=vision_tform_goal.y, goal_heading=vision_tform_goal.rot.to_yaw(),
                        frame_name=VISION_FRAME_NAME)
# Run the command instance using your robot_command_client
robot_command_client.robot_command(robot_move_to_cmd, end_time_secs=time.time() + end_time_add)

# -- End of vision transform movement command -- 



# Delay -- time.sleep()
time.sleep(end_time_add + sleep_adder)
#delay()


# -- Vision transform movement command -- 2nd go

# -- Getting robot state info --
# Get the vision_tform_body transform to understand the robot's current position in the vision frame
vision_tform_body_2 = get_vision_tform_body(
                    robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)

# -- Setting body goal --
body_tform_goal_2 = math_helpers.SE3Pose(x=1, y=0, z=0, rot=math_helpers.Quat())

# -- Setting goal for vision transform --
# Creates goal state for the vision transform by multiplying the current state (vision body) by the goal
# (body goal)
vision_tform_goal_2 = vision_tform_body_2 * body_tform_goal_2

# Move to location specified by the body and vision goal transforms
# Create command instance
robot_move_to_cmd_2 = RobotCommandBuilder.synchro_se2_trajectory_point_command(goal_x=vision_tform_goal_2.x,
                        goal_y=vision_tform_goal_2.y, goal_heading=vision_tform_goal_2.rot.to_yaw(),
                        frame_name=VISION_FRAME_NAME)
# Run the command instance using your robot_command_client
robot_command_client.robot_command(robot_move_to_cmd_2, end_time_secs=time.time() + end_time_add)

# -- End of vision transform movement command -- 2nd go



# Delay -- time.sleep()
time.sleep(end_time_add + sleep_adder)




# -- Odom Transform Movement Command --


# Get new robot state information after moving the robot. Here we are getting the transform odom_tform_body,
# which describes the robot body's position in the odom frame.
odom_tform_body = get_odom_tform_body(
                    robot_state_client.get_robot_state().kinematic_state.transforms_snapshot)



# We want to command a trajectory to go backwards one meter and to the left one meter.
# It is simple to define this trajectory relative to the body frame, since we know that will be
# just 1 meter backwards (negative-value) in the x-axis of the body and one meter left (positive-value)
# in the y-axis of the body.
# This transform is a body transform; it is a change made with the body as the reference
# But we want the transform relative to the odometry frame, which is the frame that we can access
# Easily using get_odom_tform_body
body_tform_goal = math_helpers.SE3Pose(x=1, y=0, z=0, rot=math_helpers.Quat())
# We can then transform this transform to get the goal position relative to the odom frame.
# This creates the odom transform by multiplying our state by our goal transform (essentially it changes
# the reference frame of the goal)
odom_tform_goal = odom_tform_body * body_tform_goal


# Command the robot to go to the goal point in the odom frame. The command will stop at the new
# position in the odom frame.
robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(goal_x=odom_tform_goal.x,
                goal_y=odom_tform_goal.y,
                goal_heading=odom_tform_goal.rot.to_yaw(),
                frame_name=ODOM_FRAME_NAME)
# Running the command
robot_command_client.robot_command(lease=None, command=robot_cmd,
                                       end_time_secs=time.time() + end_time_add)


# -- End of Odom Transform Movement Command --




time.sleep(end_time_add + sleep_adder)

# -- Velocity Command -- (v_x, v_y, v_rot) -- Max time seems to be 5 seconds --
robot_vel_cmd = RobotCommandBuilder.velocity_command(0.5, 0, 0.5)
robot_vel2_cmd = RobotCommandBuilder.velocity_command(-0.5, 0, 0.5)


# Trying to constantly rotate
i = 1
while i < 3:
    robot_command_client.robot_command(robot_vel_cmd, end_time_secs=time.time() + end_time_add - 5)
    time.sleep(end_time_add + sleep_adder)
    robot_command_client.robot_command(robot_vel2_cmd, end_time_secs=time.time() + end_time_add - 5)
    time.sleep(end_time_add + sleep_adder)
    i+=1

# -- End of Velocity Command -- 

'''

## --- End of Command List ---



