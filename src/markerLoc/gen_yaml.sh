#!/bin/bash

# http://wiki.ros.org/camera_calibration_parsers

cp /tmp/calibrationdata.tar.gz ~/tmp
cd ~/tmp
tar xzf calibrationdata.tar.gz
mv ost.txt ost.ini	# must have .ini extensions
rosrun camera_calibration_parsers convert ost.ini calib.yaml
echo "Can be found in ~/tmp"
