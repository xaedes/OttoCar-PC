#!/bin/sh
v4l2-ctl --all -d /dev/video1
v4l2-ctl -d /dev/video1 --list-framesizes=YUYV
