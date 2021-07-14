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

    for i in range(4):
        spot.pose(yaw=0.25)
        time.sleep(1.0)
        spot.pose(yaw=-0.25)
        time.sleep(1.0)

    spot.pose(yaw=0.0)
    time.sleep(1.0)

    for i in range(4):
        spot.pose(roll=0.25)
        time.sleep(1.0)
        spot.pose(roll=-0.25)
        time.sleep(1.0)

    spot.pose(roll=0.0)
    time.sleep(1.0)

    for i in range(4):
        spot.pose(pitch=0.25)
        time.sleep(1.0)
        spot.pose(pitch=-0.25)
        time.sleep(1.0)

    spot.pose(pitch=0.0)
    time.sleep(1.0)

    # Power down
    spot.estop(graceful=True)

    print('Trying to make Python GC the Spot object')
    spot = None
    time.sleep(5.0)

    exit(0)
