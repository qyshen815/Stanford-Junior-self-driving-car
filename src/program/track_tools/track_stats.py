#!/usr/bin/python

import os, sys, string

if(len(sys.argv[1:]) == 0):
    print 'Usage: track_stats.py TRACK_MANAGER [TRACK_MANAGER ...]'
    sys.exit()

for l in ['bicyclist', 'car', 'pedestrian', 'background', 'unlabeled']:
    os.system('echo ' + l + ': `grep -a ' + l + ' ' + string.join(sys.argv[1:], ' ') + ' | wc -l`')

