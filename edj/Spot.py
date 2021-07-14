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
from bosdyn.client.image import ImageClient
from bosdyn.util import seconds_to_duration

BELLY_RUB_RIGHT = 1
BELLY_RUB_LEFT = 2

class Spot:
    def __init__(self, project='custom_project', ip='192.168.50.3', username='student_HSD', password='dgHGcrD43SCgl', trace_level=1):
        self.trace_level = trace_level
        if trace_level >= 1:
            print('Spot module instantiated')
        # Create an sdk object (the name is arbitrary)
        self.sdk = bosdyn.client.create_standard_sdk(project)

        # Create a connection to the robot
        self.robot = self.sdk.create_robot(ip)

        # Get the client ID
        self.id_client = self.robot.ensure_client('robot-id')
        self.spot_id = self.id_client.get_id()
        if trace_level >= 1:
            print(f'Spot Id:\n{self.spot_id}')

        # Log into the robot
        self.robot.authenticate(username, password)

        # Get the robot state
        self.state_client = self.robot.ensure_client('robot-state')
        self.spot_state = self.state_client.get_robot_state()
        if trace_level >= 2:
            print(f'Spot State:\n{self.spot_state}')

        # Create an estop client and get the estop status
        self.estop_client = self.robot.ensure_client('estop')
        spot_estop_status = self.estop_client.get_status()
        if trace_level >= 2:
            print(f'Spot estop status:\n{spot_estop_status}')

        # Create an EStop end point
        self.estop_endpoint = bosdyn.client.estop.EstopEndpoint(client=self.estop_client, name='my_estop', estop_timeout=9.0)
        self.estop_endpoint.force_simple_setup()
        if trace_level >= 1:
            print('Spot estopped')

        # Spot will be estopped at this point

        # To clear the estop, you must establish a keep-alive
        self.estop_keep_alive = bosdyn.client.estop.EstopKeepAlive(self.estop_endpoint)
        spot_estop_status = self.estop_client.get_status()
        if trace_level >= 2:
            print(f'Spot estop status:\n{spot_estop_status}')
            
        # List current leases
        self.lease_client = self.robot.ensure_client('lease')
        spot_lease_list = self.lease_client.list_leases()
        if trace_level >= 2:
            print(f'Spot lease list:\n{spot_lease_list}')

        self.command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.image_client = self.robot.ensure_client(ImageClient.default_service_name)

        # Establish timesync
        self.robot.time_sync.wait_for_sync()

    @property
    def lease(self):
        return bosdyn.client.lease.LeaseKeepAlive(self.lease_client)

    def __del__(self):
        if self.trace_level >= 1:
            print('Terminating estop keep alive')
        self.estop_keep_alive.settle_then_cut()
        self.estop_keep_alive.shutdown()
        if self.trace_level >= 1:
            print('Terminating lease keep alive')
        self.lease_keep_alive.shutdown()
        if self.trace_level >= 1:
            print('Returning lease')
        self.lease_client.return_lease(self.lease)
        if self.trace_level >= 1:
            print('Spot module going out of scope')

    def power_on(self):
        # Powering Spot on
        self.robot.power_on(timeout_sec=20)
        spot_is_on = self.robot.is_powered_on()
        assert spot_is_on, "Spot failed power on"
        if spot_is_on and self.trace_level >= 1:
            print(f'Spot is powered { "up" if spot_is_on else "down" }')
        return spot_is_on

    def power_off(self, graceful=True):
        if self.trace_level >= 1:
            print('Spot powering off')
        self.robot.power_off(cut_immediately=not graceful)

    def estop(self, graceful=True):
        # EStop (cut_immediately=False will cause Spot to sit down before powering off
        # cut_immediately=True will cause power to be cut immediately, and Spot will
        # collapse)
        if graceful:
            if self.trace_level >= 1:
                print('Spot graceful estop')
            self.estop_keep_alive.settle_then_cut()
        else:
            if self.trace_level >= 1:
                print('Spot immediate estop')
            self.estop_keep_alive.stop()

    def belly_rub(self, direction=1, wait=True):
        # Belly-rub
        if self.trace_level >= 1:
            print('Spot rolling over and powering down')
        belly_rub = RobotCommandBuilder.battery_change_pose_command(dir_hint=direction) # 1 = right / 2 = left
        command_id = self.command_client.robot_command(belly_rub)
        '''
        if wait:
            now = time.time()
            while now < end_time:
                time_until_timeout = end_time - now
                rpc_timeout = max(time_until_timeout, 1)
                start_call_time = time.time()
                try:
                    response = self.command_client.robot_command_feedback(command_id, timeout=rpc_timeout)
                    mob_feedback = response.feedback.synchronized_feedback.mobility_command_feedback
                    mob_status = mob_feedback.status
                    stand_status = mob_feedback.stand_feedback.status
                except TimedOutError:
                    # Excuse the TimedOutError and let the while check bail us out if we're out of time.
                    pass
                else:
                    if mob_status != basic_command_pb2.RobotCommandFeedbackStatus.STATUS_PROCESSING:
                        raise CommandFailedError('Stand (ID {}) no longer processing (now {})'.format(
                            command_id,
                            basic_command_pb2.RobotCommandFeedbackStatus.Status.Name(mob_status)))
                    if stand_status == basic_command_pb2.StandCommand.Feedback.STATUS_IS_STANDING:
                        return
                delta_t = time.time() - start_call_time
                time.sleep(max(min(delta_t, update_time), 0.0))
                now = time.time()

            raise CommandTimedOutError(
                "Took longer than {:.1f} seconds to assure the robot stood.".format(now - start_time))
        '''

    def self_right(self, wait=True):
        if self.trace_level >= 1:
            print('Spot self-righting')
        self_right = RobotCommandBuilder.selfright_command()
        self.command_client.robot_command(self_right)

    def stand(self, wait=True):
        if self.trace_level >= 1:
            print('Spot standing')
        blocking_stand(self.command_client, timeout_sec=10)

    def pose(self, yaw=0.0, roll=0.0, pitch=0.0, wait=True):
        if self.trace_level >= 1:
            print(f'Spot posing (yaw: {yaw}, roll: {roll}, pitch: {pitch})')
        euler = bosdyn.geometry.EulerZXY(yaw=yaw, roll=roll, pitch=pitch)
        pose = RobotCommandBuilder.synchro_stand_command(footprint_R_body=euler)
        self.command_client.robot_command(pose)

    def list_image_sources(self, wait=True):
        return self.image_client.list_image_sources()

    def get_images(self, sources, wait=True):
        return self.image_client.get_image_from_sources(sources)
