#!/usr/bin/python

import sys

red = "\x1b[0;31;49m"
green = "\x1b[0;32;49m"
regular = "\x1b[0;30;49m"

break_count = 0;

for line in sys.stdin:
    if line[0:12] == '[==========]':
        break_count += 1

    if break_count > 1:
        output = ""
        if line[0:12] == '[==========]':
            output = line[13:].strip()
        elif line[0] == '[':
            output = line.strip()

        if output[0:12] == '[  PASSED  ]':
            print "       " + green + output[0:12] + " " + regular + output[13:]
        elif output[0:12] == '[  FAILED  ]':
            print "       " + red + output[0:12] + " " + regular + output[13:]
            
 
    
