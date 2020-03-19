#!/usr/bin/python

from pylab import *
import numpy
import sys
import os

if(len(sys.argv) != 2):
    print "Usage: python significance.py RUN_DIR"
    sys.exit(1)

run_dir = sys.argv[1]
if(run_dir[-1] == '/'):
    run_dir = run_dir[0:-1]
print "Computing statistics for run dir " + run_dir

filename = run_dir + '/.pylab-tmp-aotehusaotehuaoe'
os.system("grep 'Total acc' `find " + run_dir + "/* -wholename '*with/results.txt'` | awk '{print $NF}' > " + filename)
acc_with = numpy.loadtxt(filename)
os.system("rm " + filename)
os.system("grep 'Total acc' `find " + run_dir + "/* -wholename '*without/results.txt'` | awk '{print $NF}' > " + filename)
acc_without = numpy.loadtxt(filename)
os.system("rm " + filename)

print ''
print 'Frame classifier only, without canonical orientation: {0:f} +/- {1:f}'.format(numpy.mean(acc_without), numpy.std(acc_without))
#print 'Accuracy with canonical orientation: {0:f} +/- {1:f}'.format(numpy.mean(acc_with), numpy.std(acc_with))



print ''
print 'Type \t\t\t Mean \t\t Stdev'
print '------------------------------------------------------------'

for variant in ['combined', 'naive', 'evenweights', 'frame', 'pure_voting',  'global']:
    filename = run_dir + '/.pylab-tmp-asotehusatoehustaoheu'
    os.system("grep 'Total acc' `find * -name results_mll_" + variant + ".txt` | awk '{print $NF}' > " + filename)
    accuracies = numpy.loadtxt(filename)
    os.system("rm " + filename)
    print "{0:15s} \t {1:f} \t {2:f}".format(variant, numpy.mean(accuracies), numpy.std(accuracies))


