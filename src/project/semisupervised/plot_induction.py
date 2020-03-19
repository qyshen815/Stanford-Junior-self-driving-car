#!/usr/bin/python

from pylab import *
import numpy
import sys
import os
import semisupervised_common_plotting as scp

if(len(sys.argv) != 2):
    print "Usage: python plot_induction.py EPOCH_DIR"
    sys.exit(1)

epoch_dir = sys.argv[1]
print "Plotting induction information for epoch dir " + epoch_dir

# -- Config.
scp.setup()

# -- Load data.
filename = epoch_dir + '/.pylab-tmp-asotehusaocehu'

os.system("cat `find " + epoch_dir + "/* -name induction_stats.txt` | grep 'Number of inducted tracks:' | awk '{print $5}' > " + filename) # Oddly, the /* is needed or you get results in a random order.
num_inducted = numpy.loadtxt(filename)

os.system("cat `find " + epoch_dir + "/* -name induction_stats.txt` | grep 'Number of inducted tracks with labels:' | awk '{print $7}' > " + filename) 
num_inducted_with_labels = numpy.loadtxt(filename)

os.system("cat `find " + epoch_dir + "/* -name induction_stats.txt` | grep 'Number of inducted tracks with labels:' | awk '{print $7}' > " + filename) 
num_inducted_with_labels = numpy.loadtxt(filename)

os.system("cat `find " + epoch_dir + "/* -name induction_stats.txt` | grep 'Accuracy on labeled' | awk '{print $8}' > " + filename)
acc_inducted = numpy.loadtxt(filename)

#num_unlabeled_nonbg = numpy.loadtxt(sys.argv[1] + '/num_unlabeled_nonbg.txt')

# -- Draw plots.
#plot(num_inducted, scp.linestyles[4], label='Inducted tracks (all)')
plot(num_inducted_with_labels, scp.linestyles[4], label='Inducted (total)')
plot(acc_inducted * num_inducted_with_labels, scp.linestyles[0], label='Inducted (correct)')

#scp.plotInjectionEpochs(epoch_dir)
grid(True)
xlabel('Epoch')
ylabel('Number of tracks')
legend(loc='lower right')
savefig(epoch_dir + '/induction.pdf')
savefig(epoch_dir + '/induction.png')


