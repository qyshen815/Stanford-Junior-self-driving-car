#!/bin/bash

# Expects $1 to be a directory with orig/hand and orig/nonhand in it.
# Creates $1/training and $1/testing.

cd $1/orig

cd hand
mkdir -p ../../training/hand
ls | awk -v num_files=`ls | wc -l` '{if(NR < num_files / 2) print $1}' | xargs -I{} cp {} ../../training/hand/
mkdir -p ../../testing/hand
ls | awk -v num_files=`ls | wc -l` '{if(NR >= num_files / 2) print $1}' | xargs -I{} cp {} ../../testing/hand/

# Make additional rotated hands for the training set.
cd ../../training/hand
for DEG in -40 -20 20 40; do
    for file in `ls *.png | grep -v deg`; do
	echo convert $file -background black -rotate $DEG ${file%.png}_${DEG}deg.png;
	convert $file -background black -rotate $DEG ${file%.png}_${DEG}deg.png;
    done
done
cd -

cd ../nonhand
mkdir -p ../../training/nonhand
ls | awk -v num_files=`ls | wc -l` '{if(NR < num_files / 2) print $1}' | xargs -I{} cp {} ../../training/nonhand/
mkdir -p ../../testing/nonhand
ls | awk -v num_files=`ls | wc -l` '{if(NR >= num_files / 2) print $1}' | xargs -I{} cp {} ../../testing/nonhand/