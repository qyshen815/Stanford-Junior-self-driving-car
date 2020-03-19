#include <performance_statistics/performance_statistics.h>

using namespace std;
using namespace Eigen;

AccuracyHistogram::AccuracyHistogram(string xlabel, string ylabel, double bin_size, double xmin, double xmax) :
  xlabel_(xlabel),
  ylabel_(ylabel),
  bin_size_(bin_size),
  xmin_(xmin),
  xmax_(xmax)
{
  if(xmin != -FLT_MAX && xmax != FLT_MAX) { 
    double nbins = (xmax - xmin) / bin_size;
    assert(fabs(nbins - round(nbins)) < 1e-6); // Make sure that there are roughly an integral numbers of bins.  Otherwise the last bin is artificially small.
  }
}

void AccuracyHistogram::insert(int label, int prediction, double value) {
  if(value > xmax_ || value < xmin_)
    return;
  
  if(label == prediction)
    correct_.push_back(value);
  else
    incorrect_.push_back(value);
}

void AccuracyHistogram::saveHistogram(const std::string& filename) const {
  // -- Get the min and max values.
  double min = FLT_MAX;
  double max = -FLT_MAX;

  for(size_t i = 0; i < correct_.size(); ++i) {
    if(correct_[i] > max)
      max = correct_[i];
    if(correct_[i] < min)
      min = correct_[i];
  }

  for(size_t i = 0; i < incorrect_.size(); ++i) {
    if(incorrect_[i] > max)
      max = incorrect_[i];
    if(incorrect_[i] < min)
      min = incorrect_[i];
  }

  // -- Build vector of indices, starting with the minimum value of the first bin
  //    and ending with the minimum value of the last bin.
  int num_bins = ceil((max - min) / bin_size_);
  VectorXd indices(num_bins);
  for(int i = 0; i < indices.rows(); ++i) {
    indices(i) = min + i * bin_size_;
  }

  // -- Put all the data into bins.
  VectorXd correct_bins = VectorXd::Zero(indices.rows());
  for(size_t i = 0; i < correct_.size(); ++i) {
    int idx = floor((correct_[i] - min) / bin_size_);
    if(idx == num_bins) // Extreme edge case
      --idx;
    ++correct_bins(idx);
  }

  VectorXd incorrect_bins = VectorXd::Zero(indices.rows());
  for(size_t i = 0; i < incorrect_.size(); ++i) {
    int idx = floor((incorrect_[i] - min) / bin_size_);
    if(idx == num_bins)
      --idx;
    ++incorrect_bins(idx);
  }

  // -- Get percent correct.
  assert(indices.rows() == correct_bins.rows());
  assert(correct_bins.rows() == incorrect_bins.rows());
  assert(indices.rows() > 0);
  VectorXd percents = VectorXd::Zero(indices.rows());
  for(int i = 0; i < correct_bins.rows(); ++i) {
    if(correct_bins(i) + incorrect_bins(i) != 0)
      percents(i) = correct_bins(i) / (correct_bins(i) + incorrect_bins(i));
  }
  percents *= 100.0;
  
  // -- Draw and save.
  mpliBegin();
  mpli("import matplotlib.pyplot as plt");
  mpli("import numpy as np");
  mpli("import matplotlib");
  mpliPrintSize();
  
  mpliExport(indices);
  mpliExport(bin_size_);
  mpli("fig = plt.figure()");
  mpli("ax1 = fig.add_subplot(111)");
  mpli("ax2 = ax1.twinx()");

  mpliExport(correct_bins);
  mpliExport(incorrect_bins);
  mpliExport(percents);
  mpli("p1 = ax1.bar(indices, correct_bins, bin_size_, color=(0, 0.3, 0))");
  mpli("p2 = ax1.bar(indices, incorrect_bins, bin_size_, color=(0.85, 0.85, 0.85), bottom=correct_bins)");
  mpli("p3 = ax2.plot(indices + bin_size_ / 2.0, percents, 'r*--')");
  mpli("plt.legend((p1[0], p2[0], p3[0]), ('Correct', 'Incorrect', 'Percent'), loc='right')"); // Why does 'best' always suck so badly?

  mpliExport(xlabel_);
  mpliExport(ylabel_);
  mpli("ax1.set_xlabel(xlabel_, fontsize=20)");
  mpli("ax1.set_ylabel(ylabel_, fontsize=20)");
  mpli("ax2.set_ylabel('Percent', fontsize=20)");

  mpli("ax2.set_ylim(0, 100)");
  mpli("ax2.set_yticks(np.arange(20, 120, 20))");
  mpli("ax1.set_xlim(indices[0], indices[-1] + bin_size_)");
  mpli("ax2.set_xlim(indices[0], indices[-1] + bin_size_)");

  mpliExport(filename);
  mpli("plt.savefig(filename)");
  mpli("plt.clf()");
}

