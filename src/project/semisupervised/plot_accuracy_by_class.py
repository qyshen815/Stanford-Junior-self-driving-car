#!/usr/bin/python

from pylab import *
import numpy
import sys

# x = randn(10000)
# hist(x, 100) #100 bins.
# savefig('foo.pdf')
# savefig('foo.pdf')
# savefig('foo.png')
# close()

# mat = numpy.loadtxt('mat.txt')
# plot(mat[0, :], 2*mat[1, :], 'o--', label='label')
# legend()
# legend(loc='upper left')
# savefig('foo2.png')


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
car = numpy.loadtxt(sys.argv[1] + '/accuracy_by_class-car.txt')
pedestrian = numpy.loadtxt(sys.argv[1] + '/accuracy_by_class-pedestrian.txt')
bicyclist = numpy.loadtxt(sys.argv[1] + '/accuracy_by_class-bicyclist.txt')

car_baseline = ones(car.shape[0]) * numpy.loadtxt(sys.argv[1] + '/accuracy_by_class-car_baseline.txt')
pedestrian_baseline = ones(car.shape[0]) * numpy.loadtxt(sys.argv[1] + '/accuracy_by_class-pedestrian_baseline.txt')
bicyclist_baseline = ones(car.shape[0]) * numpy.loadtxt(sys.argv[1] + '/accuracy_by_class-bicyclist_baseline.txt')

# -- Draw plots.
ep = arange(car.shape[0])
plot(car, 'r-', label='car')
plot(car_baseline, 'r--', label='car baseline')
plot(bicyclist, 'g-', label='bicyclist')
plot(bicyclist_baseline, 'g--')
plot(pedestrian, 'b-', label='pedestrian')
plot(pedestrian_baseline, 'b--')
grid(True)

xlabel('Epoch')
ylabel('Accuracy')
legend(loc='lower right')
savefig(sys.argv[1] + '/accuracy_by_class.pdf')
savefig(sys.argv[1] + '/accuracy_by_class.png')


