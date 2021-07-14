#!/usr/bin/env python3

from Spot import *
import time

if __name__ == '__main__':
    spot = Spot()

    try:
        print(f'Image sources: \n{spot.list_image_sources()}')
    except:
        print('Exception')

    print('Trying to make Python GC the Spot object')
    spot = None
    time.sleep(5.0)

    exit(0)
