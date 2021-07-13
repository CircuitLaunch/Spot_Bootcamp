import bosdyn.client
import time
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand, RobotCommandBuilder

sdk = bosdyn.client.create_standard_sdk('understanding-spot')
robot = sdk.create_robot('192.168.50.3')
id_client = robot.ensure_client('robot-id')
robot.authenticate('student_HSD', 'dgHGcrD43SCgl')
state_client = robot.ensure_client('robot-state')
lease_client = robot.ensure_client('lease')
lease = lease_client.acquire()
lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(lease_client)

robot.power_on(timeout_sec=20)
print(f'Spot power: {robot.is_powered_on()}')

robot.time_sync.wait_for_sync()

command_client = robot.ensure_client(RobotCommandClient.default_service_name)
blocking_stand(command_client, timeout_sec=10)


robot_cmd = RobotCommandBuilder.velocity_command(0.5, 0, 0.5)
command_client.robot_command(robot_cmd, end_time_secs=time.time() + 2)

