import time
from bosdyn.api import robot_state_pb2, robot_command_pb2, synchronized_command_pb2, mobility_command_pb2, basic_command_pb2, geometry_pb2, trajectory_pb2
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
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME, get_vision_tform_body
from bosdyn.client import math_helpers

BELLY_RUB_RIGHT = 1
BELLY_RUB_LEFT = 2

class Spot:
    def __init__(self, project='custom_project', ip='192.168.50.3', username='student_HSD', password='dgHGcrD43SCgl', trace_level=1):
        self.trace_level = trace_level
        if trace_level >= 1:
            print('Bringing Spot Up')
        # Create an sdk object (the name is arbitrary)
        self.sdk = bosdyn.client.create_standard_sdk(project)

        # Create a connection to the robot
        self.robot = self.sdk.create_robot(ip)

        # Log into the robot
        self.robot.authenticate(username, password)

        # Get references to the clients
        self.id_client = self.robot.ensure_client('robot-id')
        self.state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.estop_client = self.robot.ensure_client('estop')
        self.lease_client = self.robot.ensure_client('lease')
        self.command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.image_client = self.robot.ensure_client(ImageClient.default_service_name)
        self.graph_nav_client = self.robot.ensure_client(GraphNavRecordingServiceClient.default_service_name)
        # self.world_object_client = self.robot.ensure_client(WorldObjectClient.default_service_name)

        # Get the client ID
        self.spot_id = self.id_client.get_id()
        if trace_level >= 2:
            print(f'Spot Id:\n{self.spot_id}')
        else:
            species = self.spot_id.species
            serial_number = self.spot_id.serial_number
            hw_version = self.spot_id.version
            major = self.spot_id.software_release.version.major_version
            minor = self.spot_id.software_release.version.minor_version
            patch = self.spot_id.software_release.version.patch_level
            print(f'{species} s/n: {serial_number}, hw version: {hw_version}, sw version: {major}.{minor} (patch level {patch})')

        # Get the robot state
        self.spot_state = self.state_client.get_robot_state()
        if trace_level >= 2:
            print(f'Spot State:\n{self.spot_state}')
        else:
            for battery_state in self.spot_state.battery_states:
                id = battery_state.identifier
                charge = battery_state.charge_percentage.value
                voltage = battery_state.voltage.value
                temperatures = battery_state.temperatures
                print(f'Battery {id} charge: {charge}, voltage: {voltage}, temperatures: {temperatures}')

        # Create an estop client and get the estop status
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
        if trace_level >= 1:
            print('Establishing estop keep-alive')
        self.estop_keep_alive = bosdyn.client.estop.EstopKeepAlive(self.estop_endpoint)
        spot_estop_status = self.estop_client.get_status()
        if trace_level >= 2:
            print(f'Spot estop status:\n{spot_estop_status}')

        # List current leases
        spot_lease_list = self.lease_client.list_leases()
        if trace_level >= 2:
            print(f'Spot lease list:\n{spot_lease_list}')

        # To obtain a lease
        if trace_level >= 1:
            print('Attempting to acquire lease')
        self.lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(self.lease_client)
        self.lease = self.lease_client.acquire()
        spot_lease_list = self.lease_client.list_leases()
        if trace_level >= 2:
            print(f'Spot lease list:\n{spot_lease_list}')
        elif trace_level >= 1:
            print('Lease acquired')

        # Construct an empty map
        self.current_graph = None
        self.current_edges = {}
        self.current_waypoint_snapshots = {}
        self.current_annotation_name_to_wp_id = {}

        # Establish timesync
        self.robot.time_sync.wait_for_sync()

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

    def wait_for_mobility_command_completion(self, command_name, command_id, completion_test, time_out=10.0, update_frequency=1.0):
        if self.trace_level >= 2:
            print(f'Waiting for completion of {command_name} command')
        now = time.time()
        end_time = now + time_out
        update_time = 1.0 / update_frequency

        while now < end_time:
            time_until_timeout = end_time - now
            rpc_timeout = max(time_until_timeout, 1)
            start_call_time = time.time()
            try:
                if self.trace_level >= 2:
                    print(f'Querying {command_name} command feedback')
                response = self.command_client.robot_command_feedback(command_id, timeout=rpc_timeout)
                if self.trace_level >= 2:
                    print(f'Attempting to access {command_name} command status')
                mob_feedback = response.feedback.synchronized_feedback.mobility_command_feedback
                mob_status = mob_feedback.status
                if self.trace_level >= 2:
                    print(f'Submitting {command_name} command status to callback for testing')
                if completion_test(mob_feedback):
                    return
            except TimedOutError:
                if self.trace_level >= 2:
                    print(f'{command_name} command timed out')
                # Excuse the TimedOutError and let the while check bail us out if we're out of time.
                pass
            except Exception as e:
                print(f'Failed to wait for status on {command_name}: {e}')
            else:
                if self.trace_level >= 2:
                    print(f'Comparing {command_name} command status')
                if mob_status != basic_command_pb2.RobotCommandFeedbackStatus.STATUS_PROCESSING:
                    raise CommandFailedError(f'{command_name} (ID {command_id}) no longer processing (now {basic_command_pb2.RobotCommandFeedbackStatus.Status.Name(mob_status)})')
            delta_t = time.time() - start_call_time
            time.sleep(max(min(delta_t, update_time), 0.0))
            now = time.time()

        raise CommandTimedOutError(
            f'Took longer than {now - start_time:.1f} seconds to complete {command_name} command.')

    def report(self, trace_level=1):
        if trace_level >= 2:
            print(f'Spot Id:\n{self.spot_id}')
        else:
            species = self.spot_id.species
            serial_number = self.spot_id.serial_number
            hw_version = self.spot_id.version
            major = self.spot_id.software_release.version.major_version
            minor = self.spot_id.software_release.version.minor_version
            patch = self.spot_id.software_release.version.patch_level
            print(f'{species} s//n: {serial_number}, hw version: {hw_version}, sw version: {major}.{minor} (patch level {patch})')

        # Get the robot state
        self.spot_state = self.state_client.get_robot_state()
        if trace_level >= 2:
            print(f'Spot State:\n{self.spot_state}')
        else:
            charge = self.spot_state.battery_states.charge_percentage.value
            voltage = self.spot_state.battery_states.voltage.value
            temperatures = self.spot_state.battery_states.temperatures
            print(f'Battery charge: {charge}, volgate: {voltage}, temperatures: {temperatures}')

        if trace_level >= 2:
            spot_estop_status = self.estop_client.get_status()
            print(f'Spot estop status:\n{spot_estop_status}')

        # List current leases
        if trace_level >= 2:
            spot_lease_list = self.lease_client.list_leases()
            print(f'Spot lease list:\n{spot_lease_list}')

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

        if wait:
            self.wait_for_mobility_command_completion('Bellyrub', command_id,
                completion_test=
                    lambda mob_feedback:
                        if self.trace_level >= 2:
                            print('Checking if battery_change_pose_feedback.status == BatteryChangePoseCommand.Feedback.STATUS_COMPLETED')
                            print(f'battery_change_pose_feedback.status == {mob_feedback.battery_change_pose_feedback.status}')
                            print(f'basic_command_pb2.BatteryChangePoseCommand.Feedback.STATUS_COMPLETED == {basic_command_pb2.BatteryChangePoseCommand.Feedback.STATUS_COMPLETED}')
                        mob_feedback.battery_change_pose_feedback.status == basic_command_pb2.BatteryChangePoseCommand.Feedback.STATUS_COMPLETED)
            return None
        return command_id

    def self_right(self, wait=True):
        if self.trace_level >= 1:
            print('Spot self-righting')
        self_right = RobotCommandBuilder.selfright_command()
        command_id = self.command_client.robot_command(self_right)

        if wait:
            self.wait_for_mobility_command_completion('Self-right', command_id,
                completion_test=
                    lambda mob_feedback:
                        mob_feedback.selfright_feedback.status == basic_command_pb2.SitCommand.Feedback.STATUS_IS_SITTING)
            return None
        return command_id

    def stand(self, wait=True):
        if self.trace_level >= 1:
            print('Spot standing')

        stand_command = RobotCommandBuilder.synchro_stand_command()
        command_id = self.command_client.robot_command(stand_command)

        if wait:
            self.wait_for_mobility_command_completion('Standing', command_id,
                completion_test=
                    lambda mob_feedback:
                        mob_feedback.stand_feedback.status == basic_command_pb2.StandCommand.Feedback.STATUS_IS_STANDING)
            return None
        return command_id

    def sit(self, wait=True):
        if self.trace_level >= 1:
            print('Spot sitting')

        sit_command = RobotCommandBuilder.synchro_sit_command()
        command_id = self.command_client.robot_command(sit_command)

        if wait:
            self.wait_for_mobility_command_completion('Sitting', command_id,
                completion_test=
                    lambda mob_feedback:
                        mob_feedback.sit_feedback.status == basic_command_pb2.SitCommand.Feedback.STATUS_IS_SITTING)
            return None
        return command_id

    def pose(self, yaw=0.0, roll=0.0, pitch=0.0, wait=True):
        if self.trace_level >= 1:
            print(f'Spot posing (yaw: {yaw}, roll: {roll}, pitch: {pitch})')
        euler = bosdyn.geometry.EulerZXY(yaw=yaw, roll=roll, pitch=pitch)
        pose = RobotCommandBuilder.synchro_stand_command(footprint_R_body=euler)
        command_id = self.command_client.robot_command(pose)

        if wait:
            self.wait_for_mobility_command_completion('Posing', command_id,
                completion_test=
                    lambda mob_feedback:
                        mob_feedback.stand_feedback.status == basic_command_pb2.StandCommand.Feedback.STATUS_IS_STANDING)
            return None
        return command_id

    def list_image_sources(self, wait=True):
        return self.image_client.list_image_sources()

    def get_images(self, sources, wait=True):
        return self.image_client.get_image_from_sources(sources)

    # Thanks to Brad "Duke Tresnor" Armstrong for figuring out trajectory locomotion
    @property
    def vision_tform_body(self):
        return get_vision_tform_body(self.state_client.get_robot_state().kinematic_state.transforms_snapshot)

    def move_to(self, x, y, z=0.0, rot_quat=math_helpers.Quat(), duration=5.0, wait=True):
        body_tform_goal = math_helpers.SE3Pose(x=x, y=y, z=0.0, rot=rot_quat)
        new_tform = self.vision_tform_body * body_tform_goal
        cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(goal_x=new_tform.x,
                        goal_y=new_tform.y, goal_heading=new_tform.rot.to_yaw(),
                        frame_name=VISION_FRAME_NAME, body_height=z)
        command_id = self.command_client.robot_command(cmd, end_time_secs=time.time() + duration)

        if wait:
            self.wait_for_mobility_command_completion('Move to', command_id,
                completion_test=
                    lambda mob_feedback:
                        mob_feedback.se2_trajectory_feedback.status == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_AT_GOAL)
            return None
        return command_id
