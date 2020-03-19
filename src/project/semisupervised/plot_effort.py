#!/usr/bin/python

from pylab import *
import numpy
import sys
import os
import semisupervised_common_plotting as scp

if(len(sys.argv) != 2):
    print "Usage: python plot_effort.py EFFORT_DIR"
    sys.exit(1)

effort_dir = sys.argv[1]
print "Plotting results for effort dir " + effort_dir


# -- Config.
scp.setup()


# -- Load data.
filename = effort_dir + '/.pylab-tmp-aosetuhasoethusatoeul'

os.system("cat " + effort_dir + "/testset_stats.txt | grep 'background tracks' | awk '{print $NF}' > " + filename)
num_background_tracks = numpy.loadtxt(filename)
os.system("cat " + effort_dir + "/testset_stats.txt | grep 'unlabeled tracks' | awk '{print $NF}' > " + filename)
num_unlabeled_tracks = numpy.loadtxt(filename)
os.system("cat " + effort_dir + "/testset_stats.txt | grep -i 'total tracks' | awk '{print $NF}' > " + filename)
num_total_tracks = numpy.loadtxt(filename)
prior_result = 100.0 * num_background_tracks / (num_total_tracks - num_unlabeled_tracks)
print "Prior classifier: " + str(prior_result)

os.system("cat `find " + effort_dir + "/ -name results.txt | sort` | grep 'Total acc' | awk '{print $NF}' > " + filename);
supervised_accuracies = 100.0 * numpy.loadtxt(filename)
print supervised_accuracies

os.system("cat `find " + effort_dir + "/ -name final_results.txt | sort` | grep 'Total acc' | awk '{print $NF}' > " + filename)
semisupervised_accuracies = 100.0 * numpy.loadtxt(filename)
print semisupervised_accuracies

os.system("echo $(for file in $(find " + effort_dir + "/ -name num_hand_labeled_tracks.txt | sort); do cat $file | awk '{sum += $NF} END {print sum}'; done) > " + filename)
num_hand_labeled_tracks = numpy.loadtxt(filename)
print num_hand_labeled_tracks


os.system("rm " + filename)



# -- Draw plots.
fig = figure()
ax1 = fig.add_subplot(111)
plot1 = ax1.plot(num_hand_labeled_tracks[0:len(supervised_accuracies)], supervised_accuracies, scp.linestyles[3], label='Supervised')
plot2 = ax1.plot(num_hand_labeled_tracks[0:len(semisupervised_accuracies)-1], semisupervised_accuracies[0:-1], scp.linestyles[1], label='Semi-supervised')
plot3 = ax1.plot(num_hand_labeled_tracks[0:-1], prior_result * ones(len(num_hand_labeled_tracks)-1), scp.linestyles[5], label='Prior only')
#plot1 = ax1.semilogx(num_hand_labeled_tracks[0:len(supervised_accuracies)], supervised_accuracies, scp.linestyles[3], label='Supervised')
#plot2 = ax1.semilogx(num_hand_labeled_tracks[0:len(semisupervised_accuracies)], semisupervised_accuracies, scp.linestyles[1], label='Semi-supervised')
#plot3 = ax1.semilogx(num_hand_labeled_tracks, prior_result * ones(len(num_hand_labeled_tracks)), scp.linestyles[5], label='Prior only')
ax1.set_xlabel('Number of hand-labeled tracks')
ax1.set_ylabel('Accuracy (\%)')
grid(True)
ymin, ymax = ylim()
ylim(ymin, 100.0)
xlim(0.9 * num_hand_labeled_tracks.min(), 1.1 * num_hand_labeled_tracks[-2])
legend(loc='best')

savefig(effort_dir + '/effort.pdf')
savefig(effort_dir + '/effort.png')
