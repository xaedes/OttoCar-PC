#!/bin/bash

# http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

echo Follow instructions at "http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration"
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.07 image:=/camera/image_raw 

