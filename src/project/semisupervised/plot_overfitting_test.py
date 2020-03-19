#!/usr/bin/env python

from pylab import *
import numpy
import sys
import os
import semisupervised_common_plotting as scp

if(len(sys.argv) != 2):
    print "Usage: python plot_accuracy.py BASELINE_DIR"
    sys.exit(1)

baseline_dir = sys.argv[1]
print "Plotting accuracy information for baseline dir " + baseline_dir

# -- Config.
scp.setup()

# -- Load data.
filename = baseline_dir + '/.pylab-tmp-asotehusatoehus'

os.system("ls baseline/overfitting_test/*-wcs.txt | sort | awk -F/ '{print $3}' | egrep -o [0-9]* > " + filename)
num_wcs = numpy.loadtxt(filename)

os.system("grep 'Total acc' `ls baseline/overfitting_test/*-wcs.txt | sort` | awk '{print $NF}' > " + filename)
accuracies = 100.0 * numpy.loadtxt(filename)

os.system("rm " + filename)

# -- Draw plots.
#fig = figure(figsize=(20,6))
#ax1 = fig.add_subplot(111)
#plot1 = ax1.plot(num_wcs, accuracies, scp.linestyles[1], label='Accuracy')
#ax1.set_xlabel('Number of weak classifiers')
#ax1.set_ylabel('Accuracy (\%)')

fig = figure(figsize=(6,2))
plot1 = plot(num_wcs, accuracies, scp.linestyles[1], label='Accuracy')
xlabel('Number of weak classifiers')
ylabel('Accuracy (\%)')
grid(True)
ymin, ymax = ylim()
ylim(85.0, 100.0)
gcf().subplots_adjust(bottom=0.22)
#legend(loc='lower right')

savefig(baseline_dir + '/overfitting_test.pdf')
savefig(baseline_dir + '/overfitting_test.png')


