#!/usr/bin/env python3

from Spot import *
import time

if __name__ == '__main__':
    spot = Spot()

    # It's ALIVE!
    spot.power_on()

    # Strike some poses
    spot.stand()
    time.sleep(5.0)

    spot.belly_rub(direction=BELLY_RUB_RIGHT)
    #time.sleep(10.0)

    spot.power_on()

    spot.self_right()
    time.sleep(10.0)

    spot.stand()
    time.sleep(5.0)

    # Power down
    spot.estop(graceful=True)

    print('Trying to make Python GC the Spot object')
    spot = None
    time.sleep(5.0)

    exit(0)
