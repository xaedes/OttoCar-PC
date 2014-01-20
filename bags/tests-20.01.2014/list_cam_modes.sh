#!/bin/bash
DEV=${1-/dev/video0}
v4l2-ctl --all -d ${DEV}
#v4l2-ctl -d ${DEV} --list-framesizes=YUYV
v4l2-ctl -d ${DEV} --list-framesizes=MJPG
