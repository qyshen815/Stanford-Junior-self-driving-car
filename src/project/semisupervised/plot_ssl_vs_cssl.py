#!/usr/bin/python

from pylab import *
import numpy
import sys
import os
import semisupervised_common_plotting as scp

if(len(sys.argv) != 2):
    print "Usage: python plot_ssl_vs_cssl.py OUTPUT_DIR"
    sys.exit(1)

output_dir = sys.argv[1]
ssl_dir = 'ssl'
cssl_dir = 'cssl'
cssl_rr_dir = 'cssl-rr'

# -- Config.
scp.setup()

# -- Load data.
filename = output_dir + '/.pylab-tmp-aotehusatoheu'
os.system("cat `find " + ssl_dir + "/* -maxdepth 1 -name timing.txt` | grep 'Total hours' | awk '{print $NF}' > " + filename) # Oddly, the /* is needed or you get results in a random order.
ssl_times = numpy.loadtxt(filename)
os.system("cat `find " + cssl_dir + "/* -maxdepth 1 -name timing.txt` | grep 'Total hours' | awk '{print $NF}' > " + filename) # Oddly, the /* is needed or you get results in a random order.
cssl_times = numpy.loadtxt(filename)
os.system("cat `find " + cssl_rr_dir + "/* -maxdepth 1 -name timing.txt` | grep 'Total hours' | awk '{print $NF}' > " + filename) # Oddly, the /* is needed or you get results in a random order.
cssl_rr_times = numpy.loadtxt(filename)
os.system("rm " + filename)

# -- Draw plots.
plot(ssl_times, scp.linestyles[0], label='SSL')
plot(cssl_times, scp.linestyles[2], label='CSSL')
plot(cssl_rr_times, scp.linestyles[1], label='CSSL-RR')
grid(True)
xlabel('Epoch')
ylabel('Time (hours)')
legend(loc='upper left')

print 'Saving to ' + output_dir + '/ssl_vs_cssl.{pdf,png}'
savefig(output_dir + '/ssl_vs_cssl.pdf')
savefig(output_dir + '/ssl_vs_cssl.png')

