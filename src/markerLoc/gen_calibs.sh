#!/bin/bash

DEV=${1-/dev/video0}
MODE=${2-MJPG}
DIR=${3-gen_calibs}
rm -rf $DIR
mkdir $DIR
echo "DO_NOT_MODIFY" > $DIR/DO_NO_MODIFY

RESULT=$(v4l2-ctl -d $DEV --list-framesizes=$MODE | sed 1d | sed 's/\t*Size: Discrete //g')
for s in $RESULT
do
	if [[ -e "calibs/$s.yaml" ]]; then
		cp "calibs/$s.yaml" $DIR/
	else
		width=$(echo $s | sed 's/x[0-9]*//')
		height=$(echo $s | sed 's/[0-9]*x//')
		cat calib_template.yaml | sed "s/@WIDTH/${width}/" | sed "s/@HEIGHT/${height}/"  > $DIR/$s.yaml
	fi
done

