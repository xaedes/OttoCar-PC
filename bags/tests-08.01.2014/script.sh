#!/bin/bash

cd data
rosbag record -a --duration=6 -x "/camera/image_raw/.+" &
#rosbag record -a --duration=29 -x "/camera/image_raw/.+" &
cd ..
cd cmd
rosbag play geradlinig.bag -s 4 -u 4 &
#rosbag play kreisfahrt.bag &
cd ..
