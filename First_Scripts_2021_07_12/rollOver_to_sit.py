import bosdyn.client
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand, RobotCommandBuilder
import time

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

# Setting up your command_client instance
command_client = robot.ensure_client(RobotCommandClient.default_service_name)

# Commanding Spot to Roll over
cmd = RobotCommandBuilder.battery_change_pose_command(dir_hint=1)
command_client.robot_command(cmd)

# Sleep command to ensure delay between roll over and self right commands
time.sleep(5)

# Commanding Spot to return to a sitting state
cmd = RobotCommandBuilder.selfright_command()
command_client.robot_command(cmd)