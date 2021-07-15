#!/usr/bin/env python3

from Spot import *
import time
from bosdyn.client import math_helpers

if __name__ == '__main__':
    spot = Spot()

    try:
        # It's ALIVE!
        spot.power_on()

        spot.move_to(1.0, 0.0, 0.0, math_helpers.Quat(), duration=5.0)

        spot.move_to(0.0, 1.0, 0.0, math_helpers.Quat(), duration=5.0)

        spot.move_to(-1.0, 0.0, 0.0, math_helpers.Quat(), duration=5.0)

        spot.move_to(0.0, -1.0, 0.0, math_helpers.Quat(), duration=5.0)

        # Power down
        spot.estop(graceful=True)
    except:
        print('Exception')

    print('Trying to make Python GC the Spot object')
    spot = None
    time.sleep(5.0)

    exit(0)
