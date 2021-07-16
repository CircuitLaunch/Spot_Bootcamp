""" Detect and follow fiducial tags. """
import cv2
import logging
import math
import numpy as np
from PIL import Image
import signal
import sys
from sys import platform
import threading
import time

from bosdyn import geometry
from bosdyn.api import image_pb2
from bosdyn.api import geometry_pb2, trajectory_pb2
from bosdyn.api import world_object_pb2
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
import bosdyn.client
from bosdyn.client import create_standard_sdk, RpcError, ResponseError
from bosdyn.client.frame_helpers import get_a_tform_b, get_vision_tform_body, BODY_FRAME_NAME, VISION_FRAME_NAME
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.lease import LeaseClient
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand
from bosdyn.client.robot_id import RobotIdClient
from bosdyn.client.robot_state import RobotStateClient
import bosdyn.client.util
from bosdyn.client.world_object import WorldObjectClient

#pylint: disable=no-member
LOGGER = logging.getLogger()

class Mover(object):

    # stuff
    def __init__(self, robot, options):
    
