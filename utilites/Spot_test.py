#!/usr/bin/env python3

from Spot import *

if __name__ == '__main__':
  spot = Spot()

	# It's ALIVE!
	spot.power_on()

	# Strike some poses
	spot.stand()
	time.sleep(5.0)

	spot.belly_rub(direction=BELLY_RUB_RIGHT)
	time.sleep(5.0)

	spot.self_right()
	time.sleep(5.0)

	spot.stand()
	time.sleep(5.0)

	# Power down
	time.estop(graceful=True)
