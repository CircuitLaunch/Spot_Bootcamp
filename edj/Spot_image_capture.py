#!/usr/bin/env python3

from Spot import *
import time

if __name__ == '__main__':
    spot = Spot()

    '''
    spot.power_on()

    spot.stand()
    time.sleep(5.0)
    '''

    list = ['right_fisheye_image', 'right_depth', 'left_fisheye_image', 'left_depth', 'frontright_fisheye_image', 'frontright_depth', 'frontleft_fisheye_image', 'frontleft_depth', 'back_fisheye_image', 'back_depth']
    images = spot.get_images(list)
    image_dict = dic(zip(list, images))
    print(image_dict)

    '''
    # Power down
    spot.estop(graceful=True)
    '''

    print('Trying to make Python GC the Spot object')
    spot = None
    time.sleep(5.0)

    exit(0)
