#!/usr/bin/env python3

from Spot import *
from bosdyn.api import image_pb2

import time
import numpy as np
import imutils
import cv2

if __name__ == '__main__':
    spot = Spot()

    try:
        list = ['right_fisheye_image', 'right_depth_in_visual_frame', 'left_fisheye_image', 'left_depth_in_visual_frame', 'frontright_fisheye_image', 'frontright_depth_in_visual_frame', 'frontleft_fisheye_image', 'frontleft_depth_in_visual_frame', 'back_fisheye_image', 'back_depth_in_visual_frame']
        images = spot.get_images(list)
        image_dict = dict(zip(list, images))
        frontright_img = None
        frontleft_img = None
        stitched = None
        for source, image_result in image_dict.items():
            print(f'From source: {source}')
            image = image_result.shot.image
            print(f'\twidth: {image.cols}')
            print(f'\theight: {image.rows}')
            num_bytes = 1
            if image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
                print('\tformat: depth_16')
                dtype = np.uint16
                extension = '.png'
            else:
                if image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
                    print('\tpixel_format: rgb_u8')
                    num_bytes = 3
                elif image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
                    print('\tpixel_format: rgba_u8')
                    num_bytes = 4
                elif image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
                    print('\tpixel_format: gs_u8')
                    num_bytes = 1
                elif image.pixel_format == image_pb2.Image.PIXEL_FORMAT_UNKNOWN:
                    print('\tpixel_format: unknown')
                else:
                    print(f'\tpixel_format: {image.pixel_format}')
                dtype = np.uint8
                extension = '.jpg'

            print('Extracting image from buffer')
            img = np.frombuffer(image.data, dtype=dtype)
            if image.format == image_pb2.Image.FORMAT_RAW:
                try:
                    print('Reshaping raw image')
                    img = img.reshape(image.rows, image.cols, num_bytes)
                except ValueError:
                    print('Decoding image')
                    img = cv2.imdecode(img, -1)
            else:
                print('Decoding image')
                img = cv2.imdecode(img, -1)

            if source[0:5] == 'front':
                print('Rotating front image')
                img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
                if source[6:11] == 'right':
                    frontleft_img = img
                if source[6:10] == 'left':
                    frontright_img = img
                if frontleft_img != None and frontright_img != None:
                    stitcher = cv2.createStitcher() if imutils.is_cv3() else cv2.Stitcher_create()
                    (status, stitched) = stitcher.stitch([frontleft_img, frontright_img])
                    if status == 0:
                        filename = f'front_stitched{extension}'
                        try:
                            cv2.imwrite(filename, stitched)
                        except:
                            print(f'Failed to write {filename}')


            elif source[0:5] == 'right':
                print('Rotating right image')
                img = cv2.rotate(img, cv2.ROTATE_180)

            filename = f'{source}{extension}'
            try:
                cv2.imwrite(filename, img)
            except:
                print(f'Failed to write {filename}')

    except ValueError as e:
        print(f'ValueError {e}')
    except Exception as e:
        print(f'Exception {e}')

    print('Trying to make Python GC the Spot object')
    spot = None
    time.sleep(5.0)

    exit(0)
