from Spot import *
import time

if __name__ == '__main__':
    spot = Spot()

    # It's ALIVE!
    spot.power_on()

    # Power down
    spot.estop(graceful=True)
    
    print('Trying to make Python GC the Spot object')
    spot = None
    time.sleep(5.0)

    exit(0)
