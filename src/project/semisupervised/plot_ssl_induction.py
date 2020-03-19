#!/usr/bin/python

from pylab import *
import numpy
import sys
import os
import semisupervised_common_plotting as scp

if(len(sys.argv) != 1):
    print "Usage: python plot_ssl_induction.py"
    sys.exit(1)

epoch_dir = 'ssl'
print "Plotting induction information for epoch dir " + epoch_dir

# -- Config.
fig_width_pt = 500.0  # Get this from LaTeX using \showthe\columnwidth
inches_per_pt = 1.0/72.27               # Convert pt to inch
golden_mean = (sqrt(5)-1.0)/2.0         # Aesthetic ratio
fig_width = fig_width_pt*inches_per_pt  # width in inches
fig_height = fig_width*golden_mean      # height in inches
fig_size =  [fig_width,fig_height]
params = {'figure.figsize': fig_size}
rcParams.update(params)

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
#plot(ones(num_inducted.shape[0]) * num_unlabeled_nonbg, '--', label='Number of non-bg unlabeled tracks')
plot(num_inducted, label='Total inducted tracks')
plot(num_inducted_with_labels, label='Consistent inducted tracks')
plot(acc_inducted * num_inducted_with_labels, label='Correctly inducted')

#scp.plotInjectionEpochs(epoch_dir)

grid(True)
xlabel('Epoch')
ylabel('Number of tracks')
legend(loc='lower right')
savefig('ssl_induction.pdf')
savefig('ssl_induction.png')


