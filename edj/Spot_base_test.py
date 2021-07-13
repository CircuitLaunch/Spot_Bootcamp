from Spot import *
import time

if __name__ == '__main__':
    spot = Spot()

    # It's ALIVE!
    spot.power_on()

    # Power down
    spot.estop(graceful=True)
