import time
from bosdyn.api import robot_state_pb2, robot_command_pb2, synchronized_command_pb2, mobility_command_pb2, basic_command_pb2, geometry_pb2, trajectory_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.api.graph_nav import graph_nav_pb2
from bosdyn.api.graph_nav import map_pb2
from bosdyn.api.graph_nav import nav_pb2
import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME, get_vision_tform_body
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.image import ImageClient
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.recording import GraphNavRecordingServiceClient
from bosdyn.client.local_grid import LocalGridClient
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME, get_vision_tform_body, get_odom_tform_body
from bosdyn.client import math_helpers
from bosdyn.util import seconds_to_duration
from threading import Thread, Lock

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

        self.id_client = self.robot.ensure_client('robot-id')
        self.state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.spot_state = self.state_client.get_robot_state()
        self.estop_client = self.robot.ensure_client('estop')
        self.lease_client = self.robot.ensure_client('lease')
        self.command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.image_client = self.robot.ensure_client(ImageClient.default_service_name)
        self.nav_client = self.robot.ensure_client(GraphNavClient.default_service_name)
        # self.nav_recording_client = self.robot.ensure_client(GraphNavRecordingServiceClient.default_service_name)

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
        self.lease_wallet = self.lease_client.lease_wallet

        self.lease = self.lease_client.acquire()
        spot_lease_list = self.lease_client.list_leases()
        if trace_level >= 2:
            print(f'Spot lease list:\n{spot_lease_list}')
        elif trace_level >= 1:
            print('Lease acquired')

        self.map_current_filepath = None
        self.current_graph = None
        self.current_edges = {}
        self.current_waypoint_snapshots = {}
        self.current_edge_snapshots = {}
        self.current_annotation_name_to_wp_id = {}

        self._nav_thread = None
        self._abort_nav_mutex = Lock()
        self._abort_nav = False
        self._current_nav_cmd_id_mutex = Lock()
        self._current_nav_cmd_id = -1

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

    '''
    @property
    def lease(self):
        self.lease = bosdyn.client.lease.LeaseKeepAlive(self.lease_client)
    '''

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

    # Thanks to Brad "Duke Tresnor" Armstrong for figuring out trajectory locomotion
    @property
    def vision_tform_body(self):
        return get_vision_tform_body(self.state_client.get_robot_state().kinematic_state.transforms_snapshot)

    @property
    def odom_tform_body(self):
        return get_odom_tform_body(self.state_client.get_robot_state().kinematic_state.transforms_snapshot).to_proto()

    def move_to(self, x, y, z, rot_quat, duration=30.0, wait=True):
        body_tform_goal = math_helpers.SE3Pose(x=x, y=y, z=z, rot=rot_quat)
        new_tform = self.vision_tform_body * body_tform_goal
        cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(goal_x=new_tform.x,
                        goal_y=new_tform.y, goal_heading=new_tform.rot.to_yaw(),
                        frame_name=VISION_FRAME_NAME)
        cmd_status = self.command_client.robot_command(cmd, end_time_secs=time.time() + duration)
        return cmd_status

    def upload_map(self, filepath):
        self.map_current_filepath = filepath
        with open(f'{filepath}/graph', 'rb') as graph_file:
            if self.trace_level >= 1:
                print(f'Reading {filepath}/graph')
            data = graph_file.read()
            self.current_graph = map_pb2.Graph()
            self.current_graph.ParseFromString(data)
        for waypoint in self.current_graph.waypoints:
            with open(f'{filepath}/waypoint_snapshots/{waypoint.snapshot_id}', 'rb') as snapshot_file:
                if self.trace_level >= 1:
                    print(f'Reading waypoint snapshot {waypoint.snapshot_id}')
                wp_snapshot = map_pb2.WaypointSnapshot()
                wp_snapshot.ParseFromString(snapshot_file.read())
                self.current_waypoint_snapshots[wp_snapshot.id] = wp_snapshot
        for edge in self.current_graph.edges:
            with open(f'{filepath}/edge_snapshots/{edge.snapshot_id}', 'rb') as snapshot_file:
                if self.trace_level >= 1:
                    print(f'Reading edge snapshot {edge.snapshot_id}')
                ed_snapshot = map_pb2.EdgeSnapshot()
                ed_snapshot.ParseFromString(snapshot_file.read())
                self.current_waypoint_snapshots[ed_snapshot.id] = ed_snapshot

        if self.trace_level >= 1:
            print(f'Uploading map at {filepath} to Spot')
        response = self.nav_client.upload_graph(lease=self.lease, graph=self.current_graph)

        for snapshot_id in response.unknown_waypoint_snapshot_ids:
            wp_snapshot = self.current_waypoint_snapshots[snapshot_id]
            self.nav_client.upload_waypoint_snapshot(wp_snapshot)
            if self.trace_level >= 1:
                print(f'Uploaded waypoint snapshot {snapshot_id}')
        for snapshot_id in response.unknown_edge_snapshot_ids:
            ed_snapshot = self.current_edge_snapshots[snapshot_id]
            self.nav_client.upload_edge_snapshot(ed_snapshot)
            if self.trace_level >= 1:
                print(f'Uploaded edge snapshot {snapshot_id}')

        print('Map upload complete')

        localization_state = self.nav_client.get_localization_state()

        if not localization_state.localization.waypoint_id:
            if self.trace_level >= 1:
                print(f'Spot is not localized. Please localize.')

        # Update waypoints and edges
        self.current_annotation_name_to_wp_id, self.current_edges = self.update_waypoints_and_edges()

    def clear_map(self):
        self.nav_client.clear_graph(lease=self.lease.lease_proto)

    def id_to_short_code(self, wp_id):
        tokens = wp_id.split('-')
        if len(tokens) > 2:
            return f'{tokens[0][0]}{tokens[1][0]}'
        return None

    def update_waypoints_and_edges(self):
        localization_id = self.nav_client.get_localization_state().localization.waypoint_id

        name_to_id = {}
        edges = {}

        short_code_to_count = {}
        waypoint_to_timestamp = []
        for waypoint in self.current_graph.waypoints:
            # Determine the timestamp that this waypoint was created at.
            timestamp = -1.0
            try:
                timestamp = waypoint.annotations.creation_time.seconds + waypoint.annotations.creation_time.nanos / 1e9
            except:
                # Must be operating on an older graph nav map, since the creation_time is not
                # available within the waypoint annotations message.
                pass
            waypoint_to_timestamp.append((waypoint.id,
                                            timestamp,
                                            waypoint.annotations.name))

            # Determine how many waypoints have the same short code.
            short_code = self.id_to_short_code(waypoint.id)

            if short_code not in short_code_to_count:
                short_code_to_count[short_code] = 0
            short_code_to_count[short_code] += 1

            # Add the annotation name/id into the current dictionary.
            waypoint_name = waypoint.annotations.name
            if waypoint_name:
                if waypoint_name in name_to_id:
                    # Waypoint name is used for multiple different waypoints, so set the waypoint id
                    # in this dictionary to None to avoid confusion between two different waypoints.
                    name_to_id[waypoint_name] = None
                else:
                    # First time we have seen this waypoint annotation name. Add it into the dictionary
                    # with the respective waypoint unique id.
                    name_to_id[waypoint_name] = waypoint.id

        # Sort the set of waypoints by their creation timestamp. If the creation timestamp is unavailable,
        # fallback to sorting by annotation name.
        waypoint_to_timestamp = sorted(waypoint_to_timestamp, key= lambda x:(x[1], x[2]))

        def pp_waypoints(waypoint_id, waypoint_name, short_code_to_count, localization_id):
            short_code = self.id_to_short_code(waypoint_id)
            if short_code is None or short_code_to_count[short_code] != 1:
                short_code = '  '  # If the short code is not valid/unique, don't show it.

            print(f'{"->" if localization_id == waypoint_id else "  "} waypoint: {waypoint_name} id: {waypoint_id} short code: {short_code}')

        print(f'{len(graph.waypoints)} waypoints:')
        for waypoint in waypoint_to_timestamp:
            pp_waypoints(waypoint[0], waypoint[2], short_code_to_count, localization_id)

        for edge in self.current_graph.edges:
            if edge.id.to_waypoint in edges:
                if edge.id.from_waypoint not in edges[edge.id.to_waypoint]:
                    edges[edge.id.to_waypoint].append(edge.id.from_waypoint)
            else:
                edges[edge.id.to_waypoint] = [edge.id.from_waypoint]

        return name_to_id, edges

    def set_initial_localization_fiducial(self):
        """Trigger localization when near a fiducial."""
        current_odom_tform_body = self.odom_tform_body
        # Create an empty instance for initial localization since we are asking it to localize
        # based on the nearest fiducial.
        localization = nav_pb2.Localization()
        self._graph_nav_client.set_localization(initial_guess_localization=localization, ko_tform_body=current_odom_tform_body)

    def find_unique_waypoint_id(self, short_code):
        name_to_id = self.current_annotation_name_to_wp_id
        if len(short_code) != 2:
            if short_code in name_to_id:
                if name_to_id[short_code] is not None:
                    return name_to_id[short_code]
                else:
                    print('{short_code} is not-unique. Please use the full waypoint-id')
                    return None
            return short_code

        ret = short_code
        for wp in self.current_graph.waypoints:
            if short_code == self.id_to_short_code(wp.id):
                if ret != short_code:
                    return short_code
                ret = waypoint.id
        return ret

    @property
    def is_powered_on(self):
        power_state = self.state_client.get_robot_state().power_state
        return (power_state.motor_power_state == power_state.STATE_ON)

    def toggle_power(self, should_power_on):
        power = self.is_powered_on
        if not power and should_power_on:
            self.power_on()
            motors_on = False
            while not motors_on:
                future = self.state_client.get_robot_state_async()
                response = future.result(timeout = 10.0)
                if response.power_state.motor_power_state == robot_state_pb2.PowerState.STATE_ON:
                    motors_on = True
                else:
                    time.sleep(0.25)
        elif power and not should_power_on:
            self.power_off()
        else:
            return power

        return self.is_powered_on

    @property
    def current_nav_cmd_id(self):
        with self._current_nav_cmd_id_mutex:
            return self._current_nav_cmd_id

    @current_nav_cmd_id.setter
    def current_nav_cmd_id(self, value):
        with self._current_nav_cmd_id_mutex:
            self._current_nav_cmd_id = value

    @property
    def nav_status(self):
        cmd_id = self.current_nav_cmd_id
        feedback = self.nav_client.navigation_feedback(cmd_id)
        return feedback.status

    @property
    def nav_finished(self):
        status = self.nav_status
        if status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
            # Successfully completed the navigation commands!
            return True
        elif status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            return True
        elif status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            return True
        elif status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED:
            return True

        # Navigation command is not complete yet.
        return False

    def report_nav_status(self):
        status = self.nav_status
        if status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
            print('Spot has reached the goal.')
        elif status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            print('Spot is lost.')
        elif status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            print('Spot encounted an obstacle.')
        elif status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED:
            print('Spot impaired.')
        else:
            print('Navigation incomplete')

    @property
    def abort_nav(self):
        with self._abort_nav_mutex:
            return self._abort_nav

    @abort_nav.setter
    def abort_nav(self, value):
        with self._abort_nav_mutex:
            self._abort_nav = value

    def nav_to(self, waypoint_id):
        self.lease = self.lease_wallet.get_lease()
        wp = self.find_unique_waypoint_id(waypoint_id, self.current_annotation_name_to_wp_id)
        if not wp:
            return
        if not self.toggle_power(should_power_on=True):
            print('Could not power up Spot')
            return

        self.lease = self.lease_wallet.advance()
        sublease = self.lease.create_sublease()
        self.lease_keep_alive.shutdown()
        is_finished = False
        while not self.nav_finished and not self.abort_nav:
            try:
                self.current_nav_cmd_id = self.nav_client.navigate_to(wp, leases=[sublease.lease_proto], command_id=nav_cmd_id)
            except ResponseError as e:
                print(f'Navigation error {e}')
                break
            time.sleep(0.5)

        self.lease = self.lease_wallet.advance()
        self.lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(self.lease_client)

    def threaded_nav_to(self, waypoint_id):
        self.abort_nav = False
        self._nav_thread = Thread(target=self.nav_to, args=(waypoint_id))
        self._nav_thread.start()

    def wait_nav_thread(self):
        self._nav_thread.join()
        self._nav_thread = None
