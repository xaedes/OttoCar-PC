#!/bin/bash

rosrun camera_calibration cameracheck.py --size 8x6 --square 0.07 /monocular:=/camera /camera/image_rect:=/camera/image_raw
