#!/usr/bin/env python3

from Spot import *
import copy

if __name__ == '__main__':
	spot = Spot()

	try:
		path_to_map = './map'
		initial_waypoint_short_code = 'sl'
		waypoint_id_or_short_code = 'zc'

		# spot.clear_map()

		spot.power_on()
		spot.stand()
		time.sleep(5.0)

		spot.upload_map(path_to_map)

		# spot.set_initial_localization_fiducial()
		# spot.set_initial_localization_waypoint(initial_waypoint_short_code)

		child_found = False

		#print('getting short_codes')
		short_codes = spot.short_codes

		# print('popping first 10 short codes')
		# sl is actually the start; the first 10 are junk waypoints
		for i in range(10):
			short_codes.pop(0)

		reverse_short_codes = copy.deepcopy(short_codes)
		reverse_short_codes.reverse()

		# print('popping first short code in reversed list')
		reverse_short_codes.pop(0)

		to_and_fro = short_codes + reverse_short_codes

		for short_code in to_and_fro:

			unique_id = spot.find_unique_waypoint_id(short_code)

			spot.threaded_nav_to(unique_id)

			spot.wait_nav_thread()

			start_time = time.time()
			while time.time() - start_time < 2.0:
				if spot.find_fiducial(220):
					spot.abort_nav = True
					child_found = True
					break

			if child_found:
				break

		if child_found:
			for i in range(3):
				spot.pose(yaw=0.25)
				time.sleep(0.5)
				spot.pose(yaw=-0.25)
				time.sleep(0.5)

		spot.pose(pitch=0.0)
		time.sleep(1.0)

		unique_id = spot.find_unique_waypoint_id(initial_waypoint_short_code)

		spot.threaded_nav_to(unique_id)

		spot.wait_nav_thread()

		status = spot.report_nav_status()

	except Exception as e:
		print(f'There was an exception: {e}')

	# Force garbage collection
	spot = None
	time.sleep(5.0)
