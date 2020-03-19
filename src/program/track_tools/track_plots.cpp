#include <track_manager.h>
#include <matplotlib_interface/matplotlib_interface.h>

using namespace std;
using namespace track_manager;
using namespace Eigen;

class Histogram
{
public:
  Histogram(string bottom_type, string top_type, string xlabel, string ylabel, double bin_size) :
    bottom_type_(bottom_type),
    top_type_(top_type),
    xlabel_(xlabel),
    ylabel_(ylabel),
    bin_size_(bin_size)
  {
    vals_.reserve(1e6);
    types_.reserve(1e6);
  }
    
  void insert(int type, double val) {
    assert(type == 0 || type == 1);

    if(vals_.size() > vals_.capacity() - 100) {
      vals_.reserve(vals_.capacity() * 2);
      types_.reserve(types_.capacity() * 2);
    }
    
    vals_.push_back(val);
    types_.push_back(type);
  }

  double getMeanValue() {
    double total = 0;
    for(size_t i = 0; i < vals_.size(); ++i)
      total += vals_[i];
    return total / (double)vals_.size();
  }

  double getMaxValue() {
    double max = -FLT_MAX;
    for(size_t i = 0; i < vals_.size(); ++i) {
      if(vals_[i] > max)
	max = vals_[i];
    }
    return max;
  }

  void setupHistogram(double* max, double* min, VectorXd* indices) {
    // -- Get the max and min values.
    *max = -FLT_MAX;
    *min = FLT_MAX;
    for(size_t i = 0; i < vals_.size(); ++i) {
      if(vals_[i] > *max)
	*max = vals_[i];
      if(vals_[i] < *min)
	*min = vals_[i];
    }
    assert(*max > -FLT_MAX);
    assert(*min < FLT_MAX);
    
    // -- Set up vector of indices, starting with minimum value of the first bin
    //    and ending with the min value of the last bin.
    int num_bins = ceil((*max - *min) / bin_size_);
    *indices = VectorXd::Zero(num_bins);
    for(int i = 0; i < indices->rows(); ++i) {
      (*indices)(i) = *min + i * bin_size_;
    }
  }

  //! Ignores the label info.  No top and bottom type.
  void savePureHistogram(string filename) {
    double max;
    double min;
    VectorXd indices;
    setupHistogram(&max, &min, &indices);

    // -- Fill the histogram.
    VectorXd hist = VectorXd::Zero(indices.rows());
    for(size_t i = 0; i < vals_.size(); ++i) {
      int idx = floor((vals_[i] - min) / bin_size_);
      if(idx == indices.rows())
	--idx;
      ++hist(idx);
    }
    
    // -- Plot.
    mpliBegin();
    mpli("import matplotlib.pyplot as plt");
    mpli("import numpy as np");
    mpli("import matplotlib");
    mpliPrintSize();
    mpli("fig = plt.figure(figsize=(8, 6))");
    mpli("ax1 = fig.add_subplot(111)");

    mpliExport(indices);
    mpliExport(bin_size_);
    mpliExport(hist);
    mpli("p1 = ax1.bar(indices, hist, bin_size_, color=(0, 0, 0.7))");


    mpliExport(xlabel_);
    mpliExport(ylabel_);
    mpli("ax1.set_xlabel(xlabel_)");
    mpli("ax1.set_ylabel(ylabel_)");
    mpli("ax1.set_xlim(indices[0], indices[-1] + bin_size_)");
    
    mpliExport(filename);
    mpli("plt.savefig(filename)");
    mpli("plt.clf()");
    mpli("matplotlib.rcdefaults()");
  }

  
  void save(string filename) {
    assert(!vals_.empty());

    double max;
    double min;
    VectorXd indices;
    setupHistogram(&max, &min, &indices);
    
    // -- Fill the histogram.
    int num_bins = indices.rows();
    VectorXd top = VectorXd::Zero(num_bins);
    VectorXd bottom = VectorXd::Zero(num_bins);
    for(size_t i = 0; i < vals_.size(); ++i) {
      int idx = floor((vals_[i] - min) / bin_size_);
      if(idx == num_bins)
	--idx;
      
      if(types_[i] == 0)
	++bottom(idx);
      else
	++top(idx);
    }
    
    // -- Plot.
    mpliBegin();
    mpli("import matplotlib.pyplot as plt");
    mpli("import numpy as np");
    mpli("import matplotlib");
    mpliPrintSize();
    
    mpliExport(indices);
    mpliExport(bin_size_);
    mpli("fig = plt.figure()");
    mpli("ax1 = fig.add_subplot(111)");

    mpliExport(top);
    mpliExport(bottom);
    mpliExport(top_type_);
    mpliExport(bottom_type_);
    mpli("p1 = ax1.bar(indices, bottom, bin_size_, color='0.25')");
    mpli("p2 = ax1.bar(indices, top, bin_size_, color='red', bottom=bottom)");
    mpli("plt.legend((p1[0], p2[0]), (bottom_type_, top_type_), loc='best')"); // Why does 'best' always suck so badly?

    mpliExport(xlabel_);
    mpliExport(ylabel_);
    mpli("ax1.set_xlabel(xlabel_)");
    mpli("ax1.set_ylabel(ylabel_)");
    mpli("ax1.set_xlim(indices[0], indices[-1] + bin_size_)");
    
    mpliExport(filename);
    mpli("plt.savefig(filename)");

    mpli("plt.clf()");
    mpli("matplotlib.rcdefaults()");
  }

private:
  string bottom_type_;
  string top_type_;
  string xlabel_;
  string ylabel_;
  double bin_size_;
  vector<double> vals_;
  //! Aligned with vals_.
  vector<int> types_;
};


string usageString() {
  ostringstream oss;
  oss << "Usage: " << endl;
  oss << " track_plots TRACK_MANAGER [TRACK_MANAGER ...] OUTPUT_STEM" << endl;
  return oss.str();
}


void accumulate(TrackManager& tm, Histogram* track_hist, Histogram* frame_hist) {
  for(size_t i = 0; i < tm.tracks_.size(); ++i) {
    // -- Ignore unlabeled.
    Track& track = *tm.tracks_[i];
    if(track.label_.compare("unlabeled") == 0)
      continue;

    // -- Get distance and label.
    double dist = track.getMeanDistance();
    int is_foreground = 1;
    if(track.label_.compare("background") == 0)
      is_foreground = 0;

    // -- Update track stats.
    track_hist->insert(is_foreground, dist);

    // -- Update frame stats.
    for(size_t j = 0; j < track.frames_.size(); ++j)
      frame_hist->insert(is_foreground, track.frames_[j]->getDistance(track.velodyne_offset_));
  }
}

int main(int argc, char** argv) {
  if(argc > 1) {
    string output_stem(argv[argc - 1]);
    if(output_stem.find(".tm") != string::npos) {
      cout << usageString() << endl;
      return 1;
    }

    vector<string> filenames;
    for(int i = 1; i < argc - 1; ++i)
      filenames.push_back(argv[i]);

    Histogram track_hist("Background", "Foreground", "Mean distance (meters)", "Number of tracks", 10);
    Histogram frame_hist("Background", "Foreground", "Distance (meters)", "Number of segments", 10);
    for(size_t i = 0; i < filenames.size(); ++i) {
      cout << "Working on " << filenames[i] << ", " << i+1 << " / " << filenames.size() << endl;
      TrackManager tm(filenames[i]);
      accumulate(tm, &track_hist, &frame_hist);
    }
    cout << "Mean distance to frame is: " << frame_hist.getMeanValue() << endl;
    cout << "Max distance to frame is: " << frame_hist.getMaxValue() << endl;
    cout << "Plotting..." << endl;
    
    track_hist.save(output_stem + "_track.png");
    track_hist.save(output_stem + "_track.pdf");
    cout << "Saved to " << output_stem + "_track.png" << " and " << output_stem + "_track.pdf" << endl;

    frame_hist.save(output_stem + "_frame.png");
    frame_hist.save(output_stem + "_frame.pdf");
    cout << "Saved to " << output_stem + "_frame.png" << " and " << output_stem + "_frame.pdf" << endl;

    track_hist.savePureHistogram(output_stem + "_purehistogram_track.png");
    track_hist.savePureHistogram(output_stem + "_purehistogram_track.pdf");

    frame_hist.savePureHistogram(output_stem + "_purehistogram_frame.png");
    frame_hist.savePureHistogram(output_stem + "_purehistogram_frame.pdf");
  }

  else { 
    cout << usageString() << endl;
  }
  
  
}
