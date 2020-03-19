#!/usr/bin/python

from pylab import *
import numpy
import sys
import os
import semisupervised_common_plotting as scp

if(len(sys.argv) != 2):
    print "Usage: python plot_accuracy_vs_quantity.py QUANTITY_DIR"
    sys.exit(1)

quant_dir = sys.argv[1]
print "Plotting accuracy information for quantity dir " + quant_dir

# -- Config.
scp.setup()

# -- Load data.
filename = quant_dir + '/.pylab-tmp-aodeusaogedgqdjkth'
os.system("cat `find " + quant_dir + "/* -name final_results.txt` | grep accuracy | awk '{print $NF}' > " + filename)
accuracies = 100.0 * numpy.loadtxt(filename)

os.system("rm " + filename)
os.system("for dir in `find " + quant_dir +
          """/* -maxdepth 0 -type d`; do cat $dir/`ls $dir | grep epoch | awk 'END {print $0}'`/induction_stats.txt |
          grep 'treated as unlabeled' | awk '{print $NF}' >> """ +
          filename + "; done")
num_unlabeled = numpy.loadtxt(filename)

os.system("rm " + filename)
os.system("for dir in `find " + quant_dir +
          """/* -maxdepth 0 -type d`; do cat $dir/`ls $dir | grep epoch | awk 'END {print $0}'`/induction_stats.txt |
          grep 'Number of inducted tracks:' | awk '{print $NF}' >> """ +
          filename + "; done")
num_inducted = numpy.loadtxt(filename)

os.system("cat " + quant_dir + "/baseline_results.txt | grep accuracy | awk '{print $8}' > " + filename)
baseline_accuracy = 100.0 * numpy.loadtxt(filename)
os.system("rm " + filename)

percents_inducted = numpy.loadtxt(quant_dir + '/percents_inducted.txt')

# -- Draw basic plot.
fig = figure()
ax1 = fig.add_subplot(111)
plot1 = ax1.plot(num_unlabeled, accuracies, scp.linestyles[1])
plot2 = ax1.plot(num_unlabeled, baseline_accuracy * ones(len(accuracies)), scp.linestyles[3], label='Baseline accuracy')
ax1.set_xlabel('Number of unlabeled tracks provided')
ax1.set_ylabel('Final accuracy (\%)')
grid(True)
ymin, ymax = ylim()
ylim(ymin, 100)

ax2 = ax1.twinx()
plot3 = ax2.plot(num_unlabeled, percents_inducted, 'g-', label='\% inducted')
ax2.set_ylabel('Unlabeled tracks inducted (\%)')
ymin, ymax = ylim()
ylim(ymin, ymax * 1.2)

legend([plot1, plot2, plot3],['Semi-supervised', 'Supervised', '\% inducted'], loc='lower right')

savefig(quant_dir + '/accuracy.pdf')
savefig(quant_dir + '/accuracy.png')

# -- Draw log-log plot.
fig = figure()
ax1 = fig.add_subplot(111)
plot1 = ax1.loglog(num_unlabeled, 100.0 * ones(accuracies.shape) - accuracies, scp.linestyles[3])
plot2 = ax1.plot(num_unlabeled, 100.0 * ones(accuracies.shape) - (baseline_accuracy * ones(len(accuracies))), scp.linestyles[1], label='Baseline accuracy')
ax1.set_xlabel('Number of unlabeled tracks')
ax1.set_ylabel('Error')
grid(True)
ymin, ymax = ylim()
#ylim(ymin, 1.0)
xmin, xmax = xlim()
#xlim(num_unlabeled[0], num_unlabeled[-1])

#ax2 = ax1.twinx()
#plot3 = ax2.plot(num_unlabeled, num_inducted, 'g-', label='Number of inducted tracks')
#ax2.set_ylabel('Number of inducted tracks')

#scp.plotInjectionEpochs(epoch_dir)

#legend([plot1],['Final accuracy'], loc='lower right')

savefig(quant_dir + '/accuracy_loglog.pdf')
savefig(quant_dir + '/accuracy_loglog.png')

