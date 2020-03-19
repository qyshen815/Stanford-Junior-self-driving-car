#!/bin/python

import sys
import os

if(len(sys.argv) != 3):
    print "Usage: python format_calibration_output.py CAMERA_NAME MESSAGE"
    sys.exit(1)

camera_name = sys.argv[1]
filename = sys.argv[2]

f = open(filename, 'r')
height = -1
width = -1
for line in f:
    if(line[0:6] == '[INFO]'):
        continue
    elif(line[0:6] == 'height'):
        height = int(line[8:-1])
    elif(line[0:5] == 'width'):
        width = int(line[7:-1])
    elif(line[0:3] == 'R: '):
        rectification_matrix = line[3:-1]
    elif(line[0:3] == 'D: '):
        distortion_coefficients = line[3:-1]
    elif(line[0:3] == 'P: '):
        projection_matrix = line[3:-1]
    elif(line[0:3] == 'K: '):
        camera_matrix = line[3:-1]
    
print 'image_width: ' + str(width)
print 'image_height: ' + str(height)
print 'camera_name: ' + camera_name
print 'camera_matrix: '
print '  rows: 3'
print '  cols: 3'
print '  data: ' + camera_matrix
print 'distortion_coefficients: '
print '  rows: 1'
print '  cols: 5'
print '  data: ' + distortion_coefficients
print 'rectification_matrix: '
print '  rows: 3'
print '  cols: 3'
print '  data: ' + rectification_matrix
print 'projection_matrix: '
print '  rows: 3'
print '  cols: 4'
print '  data: ' + projection_matrix
