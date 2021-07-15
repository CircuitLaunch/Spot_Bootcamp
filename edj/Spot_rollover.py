#!/usr/bin/env python3

from Spot import *
import time

if __name__ == '__main__':
    spot = Spot(trace_level=2)

    try:
        # It's ALIVE!
        spot.power_on()

        spot.belly_rub(direction=BELLY_RUB_RIGHT)

        # Power down
        spot.estop(graceful=True)
    except:
        print('Exception')

    print('Trying to make Python GC the Spot object')
    spot = None
    time.sleep(5.0)

    exit(0)
