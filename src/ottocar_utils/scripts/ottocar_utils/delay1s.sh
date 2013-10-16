#!/bin/bash

# Usage: delay1s.sh COMMAND
# executes COMMAND (with all arguments given) after one second
# is used to execute arbitrary commands with roslaunch

sleep 1
"$@"