#!/usr/bin/env python3

from Spot import *

if __name__ == '__main__':
	spot = Spot()

	try:
		path_to_map = './map'
		waypoint_id_or_short_code = 'fm'

		spot.clear_map()

		spot.upload_map(my_map_path)

		unique_id = spot.find_unique_waypoint_id(waypoint_id_or_short_code)

		spot.threaded_nav_to(unique_id)

		spot.wait_nav_thread()

		status = spot.report_nav_status()

	except Exception as e:
		print('There was an exception: {e}')

	# Force garbage collection
	spot = None
	time.sleep(5.0)
