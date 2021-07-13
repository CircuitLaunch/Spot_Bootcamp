import bosdyn.client, json
sdk = bosdyn.client.create_standard_sdk('understanding-spot')
robot = sdk.create_robot('192.168.1.196')
id_client = robot.ensure_client('robot-id')
#print(id_client.get_id())
robot.authenticate('student_HSD', 'dgHGcrD43SCgl')
state_client = robot.ensure_client('robot-state')
state = state_client.get_robot_state()
print(state.battery_states)
