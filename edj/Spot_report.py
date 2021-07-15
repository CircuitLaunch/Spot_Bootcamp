#!/usr/bin/env python3

import socket
from Spot import *
import time

SpotCORE_IP = '192.168.1.195'
SpotInternal_IP = '192.168.50.3'
SpotExternal_IP = '192.168.1.196'

if __name__ == '__main__':

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('8.8.8.8', 80))
    host_ip = s.getsockname()[0]

    if host_ip == SpotCORE_IP:
        robot_ip = SpotInternal_IP
    else:
        robot_ip = SpotExternal_IP

    spot = Spot(ip=robot_ip, trace_level=1)

    print('Trying to make Python GC the Spot object')
    spot = None
    time.sleep(5.0)

    exit(0)
