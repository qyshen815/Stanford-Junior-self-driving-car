#!/usr/bin/python

from pylab import *
import numpy
import sys
import os
import semisupervised_common_plotting as scp

if(len(sys.argv) != 1):
    print "Usage: python plot_frame_induction.py"
    sys.exit(1)

epoch_dir = 'frame'
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
#plot(num_inducted, scp.linestyles[4], label='Inducted frames (all)')
#plot(num_inducted_with_labels, scp.linestyles[2], label='Inducted frames (consistent only)')
#plot(acc_inducted * num_inducted_with_labels, scp.linestyles[1], label='Correctly inducted (consistent only)')
plot(num_inducted_with_labels, scp.linestyles[4], label='Inducted (total)')
plot(acc_inducted * num_inducted_with_labels, scp.linestyles[0], label='Inducted (correct)')


#scp.plotInjectionEpochs(epoch_dir)

# Add a bit more space for the legend.
ymin, ymax = ylim()
ylim(ymin, ymax * 1.2)

grid(True)
xlabel('Epoch')
ylabel('Number of frames')
legend(loc='upper left')
savefig('frame_induction.pdf')
savefig('frame_induction.png')


