#ifndef COLOR_HISTOGRAM_NPG_H
#define COLOR_HISTOGRAM_NPG_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dst/node_potential_generator.h>

namespace dst
{

  //! Class for computing node potentials based on the color model from the seed labels.
  class ColorHistogramNPG : public NodePotentialGenerator
  {
  public:
    typedef std::vector< std::vector< std::vector<double> > > Hist;
    
    ColorHistogramNPG(pipeline2::Outlet<cv::Mat3b>* image_otl,
		      pipeline2::Outlet<cv::Mat1b>* seed_otl,
		      pipeline2::Outlet<cv::Mat3b>* prev_image_otl,
		      pipeline2::Outlet<cv::Mat1b>* prev_seg_otl,
		      double smoothing = 1.0,
		      int num_bins = 8);
    //! The default behavior is to accumulate color information over time.
    //! You must call this method if accumulation is not desired.
    void clearHistograms();
    
  protected:
    pipeline2::Outlet<cv::Mat3b>* image_otl_;
    pipeline2::Outlet<cv::Mat1b>* seed_otl_;
    pipeline2::Outlet<cv::Mat3b>* prev_image_otl_;
    pipeline2::Outlet<cv::Mat1b>* prev_seg_otl_;
    double smoothing_;
    int num_bins_;
    uchar num_vals_per_bin_;
    Hist source_hist_;
    Hist sink_hist_;

    void initializeHistogram(Hist* hist) const;
    void fillHistogram(cv::Mat1b seed, cv::Mat3b img,
		       int label, Hist* hist) const;
    int getIdx(uchar val) const;
    void _compute();
    void _display() const;
    void _flush();
    void _reset();
    std::string _getName() const;
  };
  
}

#endif // COLOR_HISTOGRAM_NPG_H
