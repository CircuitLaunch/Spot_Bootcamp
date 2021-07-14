#!/usr/bin/env python3

from Spot import *
from bosdyn.api import image_pb2

import time
import numpy as np
import cv2

if __name__ == '__main__':
    spot = Spot()

    list = ['right_fisheye_image', 'right_depth', 'left_fisheye_image', 'left_depth', 'frontright_fisheye_image', 'frontright_depth', 'frontleft_fisheye_image', 'frontleft_depth', 'back_fisheye_image', 'back_depth']
    images = spot.get_images(list)
    image_dict = dict(zip(list, images))

    for source, image_result in image_dict.items():
        print(f'From source: {source}')
        image = image_result.shot.image
        print(f'\twidth: {image.cols}')
        print(f'\theight: {image.rows}')
        if image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
            print('\tformat: depth_16')
            dtype = np.uint16
        else:
            print('\tformat: rgb_8')
            dtype = np.uint8

        img = np.fromstring(image.data, dtype=dtype)
        print(img)
        if image.pixel_format == image_pb2.Image.FORMAT_RAW:
            print('Reshaping')
            img = img.reshape(image.rows, image.cols)
        else:
            print('Decoding')
            img = cv2.imdecode(img, -1)

        print(img)
        filename = f'{source}.png'
        cv2.imwrite(filename, img)

    print('Trying to make Python GC the Spot object')
    spot = None
    time.sleep(5.0)

    exit(0)
