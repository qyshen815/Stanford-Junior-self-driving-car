#!/usr/bin/python

from pylab import *
import numpy
import sys
import os
import semisupervised_common_plotting as scp

if(len(sys.argv) != 2):
    print "Usage: python plot_timing.py EPOCH_DIR"
    sys.exit(1)

epoch_dir = sys.argv[1]
print "Plotting timing information for epoch dir " + epoch_dir

# -- Config.
scp.setup()

# -- Load data.
filename = epoch_dir + '/.pylab-tmp-aloecgusatq'
os.system("cat `find " + epoch_dir + "/* -name timing.txt` | grep 'classifier:' | awk '{print $5}' > " + filename) # Oddly, the /* is needed or you get results in a random order.
training_times = numpy.loadtxt(filename)
os.system("cat `find " + epoch_dir + "/* -name timing.txt` | grep mine | awk '{print $6}' > " + filename)
mining_times = numpy.loadtxt(filename)
os.system("rm " + filename)

# -- Draw plots.
plot(training_times / 60., label='Training')
plot(mining_times / 60., label='Mining')

scp.plotInjectionEpochs(epoch_dir)

grid(True)
xlabel('Epoch')
ylabel('Time (hours)')
legend(loc='upper left')
savefig(epoch_dir + '/timing.pdf')
savefig(epoch_dir + '/timing.png')
