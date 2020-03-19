#!/usr/bin/python

from pylab import *
import numpy
import sys
import os
import semisupervised_common_plotting as scp

if(len(sys.argv) != 2):
    print "Usage: python plot_accuracy.py EPOCH_DIR"
    sys.exit(1)

epoch_dir = sys.argv[1]
print "Plotting accuracy information for epoch dir " + epoch_dir


# -- Config.
scp.setup()

# -- Load data.
filename = epoch_dir + '/.pylab-tmp-aohedihotehukl'
os.system("cat `find " + epoch_dir + "/* -name results.txt` | grep accuracy | awk '{print $8}' > " + filename)
accuracies = 100.0 * numpy.loadtxt(filename)
os.system("cat `find " + epoch_dir + "/* -name induction_stats.txt` | grep 'Number of inducted tracks with labels:' | awk '{print $NF}' > " + filename)
num_inducted = numpy.loadtxt(filename)
os.system("cat " + epoch_dir + "/baseline_results.txt | grep accuracy | awk '{print $NF}' > " + filename)
baseline_accuracy = 100.0 * numpy.loadtxt(filename)
os.system("rm " + filename)

# -- Draw plots.
fig = figure()
ax1 = fig.add_subplot(111)
plot1 = ax1.plot(accuracies, scp.linestyles[1], label='Accuracy')
plot2 = ax1.plot(baseline_accuracy * ones(len(accuracies)), scp.linestyles[2], label='Baseline accuracy')
ax1.set_xlabel('Epoch')
ax1.set_ylabel('Accuracy (\%)')
grid(True)
ymin, ymax = ylim()
ylim(ymin, 100.0)

ax2 = ax1.twinx()
plot3 = ax2.plot(num_inducted, scp.linestyles[4], label='Number of inducted tracks')
ax2.set_ylabel('Number of inducted tracks')

# Move the inducted tracks line down a bit.
ymin, ymax = ylim()
ylim(ymin, ymax * 1.2)


#scp.plotInjectionEpochs(epoch_dir)

legend([plot1, plot2, plot3],['Semi-supervised accuracy', 'Supervised accuracy', 'Number of inducted tracks'], loc='lower right')

savefig(epoch_dir + '/accuracy.pdf')
savefig(epoch_dir + '/accuracy.png')

print 'Note: only tracks *with* labels are considered in the induction line.'

