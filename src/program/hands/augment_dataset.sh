#!/bin/bash

# Run this program in a directory with png images.
# New images with -rotated<deg>-scaled<factor>.png will be created.

cd $1
for file in `ls *.png | grep -v rotated | grep -v scaled`; do
    echo Working on $file
    for SCALE in 70 100 130; do
	convert -quality 1 $file -scale ${SCALE}% ${file%.png}-scaled${SCALE}.png
	for DEG in -30 -15 15 30; do
	    convert -quality 1 ${file%.png}-scaled${SCALE}.png -background black -rotate $DEG -gravity Center -extent 32x32 -sharpen 2x1 ${file%.png}-scaled${SCALE}-rotated${DEG}.png;
	done
	rm ${file%.png}-scaled${SCALE}.png
    done
done
cd -