#!/bin/bash

# get directory of this file (http://stackoverflow.com/a/246128/798588)
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$DIR"

rm -rf install/ build/ devel/

RSDIR="$( rospack find rosserial_arduino )/.."
$RSDIR/remake.sh

source install/setup.bash

rm -rf ~/ino/lib/ros_lib
rosrun rosserial_arduino make_libraries.py ~/ino/lib
