#!/bin/bash

cd data
# rosbag record -a --duration=6 -x "/camera/.+" &
rosbag record -a --duration=430 -x "/camera/.+" &  # record everything except additional image formats
# rosbag record -a --duration=430 -x "/camera/image_raw/.+" &  # record everything except additional image formats
cd ..
# cd cmd
# rosbag play geradlinig.bag -s 4 -u 4 &
#rosbag play kreisfahrt.bag &
# rosbag play einparken.bag -u 24 &
sleep 2
time ./steuerung.py &

# cd ..
wait
# cd cmd
# rosbag play geradlinig.bag -s 4 -u 4 &
#rosbag play kreisfahrt.bag &
# rosbag play einparken_zurueck.bag -u 7 &
# cd ..
# wait