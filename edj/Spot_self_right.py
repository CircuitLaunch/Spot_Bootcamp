#!/usr/bin/env python3

from Spot import *
import time

if __name__ == '__main__':
    spot = Spot()

    # It's ALIVE!
    spot.power_on()

    spot.self_right()
    time.sleep(10.0)

    # Power down
    spot.estop(graceful=True)
