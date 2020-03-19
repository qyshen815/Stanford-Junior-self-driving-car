#!/bin/bash

# Run this program in a directory with png images.
# New images with -rotated<deg>-scaled<factor>.png will be created.

cd $1
for file in `ls *.png | grep -v rotated | grep -v scaled`; do
    echo Working on $file
    for DEG in -30 -15 15 30; do
	convert -quality 1 $file -background black -rotate $DEG -gravity Center -extent 32x32 -sharpen 2x1 ${file%.png}-rotated${DEG}.png;
    done
done
cd -