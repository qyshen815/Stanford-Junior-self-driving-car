#!/usr/bin/python

from pylab import *
import numpy
import sys
import os

linestyles = ['ko-', 'rv--', 'bs:', 'b*--', 'g+-', 'kh:']

def getInjectionEpochs(epoch_dir):
    os.system("find " + epoch_dir + "/elicited_labels/* | grep -o [0-9]* > " + epoch_dir + "/injection_epochs.txt")
    injection_epochs = numpy.loadtxt(epoch_dir + "/injection_epochs.txt")
    return injection_epochs[:-1] # The last one isn't used.

def plotInjectionEpochs(epoch_dir):
    if(not os.path.isdir(epoch_dir + "/elicited_labels")):
        return
    
    injection_epochs = getInjectionEpochs(epoch_dir)
    scatter(injection_epochs, numpy.zeros(injection_epochs.shape), marker='x', s=40, label='Active learning') 
    for epoch in injection_epochs:
        axvline(x=epoch, linestyle='--', color='k')

def setup():
    fig_width_pt = 500.0  # Get this from LaTeX using \showthe\columnwidth
    inches_per_pt = 1.0/72.27               # Convert pt to inch
    golden_mean = (sqrt(5)-1.0)/2.0         # Aesthetic ratio
    fig_width = fig_width_pt*inches_per_pt  # width in inches
    fig_height = fig_width*golden_mean      # height in inches
    fig_size =  [fig_width,fig_height]
    params = {'figure.figsize': fig_size}
    rcParams['lines.linewidth'] = 3

    rcParams['font.size'] = 14
    rcParams['text.usetex'] = True
    rcParams.update(params)
