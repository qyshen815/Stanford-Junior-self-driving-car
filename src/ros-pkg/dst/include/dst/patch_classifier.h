#ifndef PATCH_CLASSIFIER_H
#define PATCH_CLASSIFIER_H

#include <stdint.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include <unconstrained_optimization/common_functions.h>
#include <dst/helper_functions.h>
#include <dst/node_potential_generator.h>
#include <dst/image_processing_nodes.h>

namespace dst
{
  class PatchClassifierData
  {
  public:
    PatchClassifierData();
    PatchClassifierData(cv::Mat3b bgr, cv::Mat1b intensity,
			cv::Mat1i integral, cv::Mat3f hsv);
    cv::Mat3b bgr_;
    cv::Mat1b intensity_;
    cv::Mat1i integral_;
    cv::Mat3f hsv_;
  };
  
  class BinaryFeature
  {
  public:
    typedef boost::shared_ptr<BinaryFeature> Ptr;
    BinaryFeature(int radius);
    virtual ~BinaryFeature() {}
    virtual bool compute(PatchClassifierData& data, const cv::Point2i& pt) const = 0;
    virtual std::string status() const;

  protected:
    int radius_;
    int window_width_;

    cv::Point2i getRandomPoint() const;
  };

  class PointColorThreshold : public BinaryFeature
  {
  public:
    typedef boost::shared_ptr<PointColorThreshold> Ptr;
    
    PointColorThreshold(int radius);
    bool compute(PatchClassifierData& data, const cv::Point2i& pt) const;
    std::string status() const;

  protected:
    cv::Point2i pt_;
    int channel_;
    int threshold_;
    int sign_;
  };

  class IntensityDifference : public BinaryFeature
  {
  public:
    typedef boost::shared_ptr<IntensityDifference> Ptr;

    IntensityDifference(int radius);
    bool compute(PatchClassifierData& data, const cv::Point2i& pt) const;
    std::string status() const;
    
  protected:
    cv::Point2i pt0_;
    cv::Point2i pt1_;
  };

  class ColorDifference : public BinaryFeature
  {
  public:
    typedef boost::shared_ptr<ColorDifference> Ptr;

    ColorDifference(int radius);
    bool compute(PatchClassifierData& data, const cv::Point2i& pt) const;
    std::string status() const;
    
  protected:
    cv::Point2i pt0_;
    cv::Point2i pt1_;
    int channel0_;
    int channel1_;
  };

  class HaarWavelet : public BinaryFeature
  {
  public:
    typedef boost::shared_ptr<HaarWavelet> Ptr;

    HaarWavelet(int radius);
    bool compute(PatchClassifierData& data, const cv::Point2i& pt) const;
    std::string status() const;
    
  protected:
    int evaluateBox(cv::Mat1i integral, const cv::Point2i ul, int width, int height) const;
    
    //! 0 -> vertical
    //! 1 -> horizontal
    int orientation_;
    int sign_;
    int width_;
    int height_;
    //! upper left
    cv::Point2i ul0_;
    cv::Point2i ul1_;
  };

  class Fern
  {
  public:
    // static const int NUM_FEATURES = 10;
    // static const int NUM_BINS = 1024;
    static const int NUM_FEATURES = 12;
    static const int NUM_BINS = 4096;
    typedef uint16_t FeatureVector;
    
    typedef boost::shared_ptr<Fern> Ptr;
    typedef Eigen::Matrix<float, NUM_BINS, 1> BinData;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Fern(int radius);
    virtual ~Fern();

    //! You must make sure pt is at least radius_ away from the img.
    void train(PatchClassifierData& data, const cv::Point2i& pt, uchar label);
    void train(const FeatureVector& f, uchar label);
    //! You must make sure pt is at least radius_ away from the img.
    float classify(PatchClassifierData& data, const cv::Point2i& pt) const;
    float classify(const FeatureVector& f) const;
    virtual FeatureVector computeFeatures(PatchClassifierData& data, const cv::Point2i& pt) const;
    int getRadius() const { return radius_; }
    std::string status() const;
    
  protected:
    std::vector<BinaryFeature::Ptr> features_;
    int radius_;
    int width_;
    BinData positive_;
    BinData negative_;
    BinData total_;
    BinData output_;

    void initialize();
  };
  
  class PatchClassifier
  {
  public:
    static const int NUM_FERNS = 50;
    typedef boost::shared_ptr<PatchClassifier> Ptr;
    typedef cv::Mat_< cv::Vec<Fern::FeatureVector, NUM_FERNS> > FeatureCache;
    
    PatchClassifier(int radius);
    void train(PatchClassifierData& data, const cv::Point2i& pt, uchar label);
    void train(FeatureCache& cache, const cv::Point2i& pt, uchar label);
    void setOffset(float offset) { offset_ = offset; }
    void setOffsetByPrior();
    void learnOffset(cv::Mat1f cached_classifications, cv::Mat1b labels);
    float classify(PatchClassifierData& data, const cv::Point2i& pt) const;
    float classify(FeatureCache& cache, const cv::Point2i& pt) const;
    void cacheFeatures(PatchClassifierData& data, FeatureCache& cache) const;
    int getRadius() const { return radius_; }
    std::string status() const;
    
  protected:
    std::vector<Fern::Ptr> ferns_;
    int radius_;
    float total_pos_;
    float total_neg_;
    float offset_;
  };

  class PatchClassifierNPG : public NodePotentialGenerator
  {
  public:
    PatchClassifierNPG(pipeline2::Outlet<cv::Mat3b>* img_otl,
		       pipeline2::Outlet<cv::Mat1b>* intensity_otl,
		       pipeline2::Outlet<cv::Mat1i>* integral_otl,
		       pipeline2::Outlet<cv::Mat3f>* hsv_otl,
		       pipeline2::Outlet<cv::Mat1b>* prev_seg_otl,
		       int radius);
    
  protected:
    pipeline2::Outlet<cv::Mat3b>* img_otl_;
    pipeline2::Outlet<cv::Mat1b>* intensity_otl_;
    pipeline2::Outlet<cv::Mat1i>* integral_otl_;
    pipeline2::Outlet<cv::Mat3f>* hsv_otl_;
    pipeline2::Outlet<cv::Mat1b>* prev_seg_otl_;
    int num_ferns_;
    int radius_;
    PatchClassifier::Ptr classifier_;
    PatchClassifierData data_;
    PatchClassifierData prev_data_;
    cv::Mat1f cached_classifications_;
    PatchClassifier::FeatureCache cached_features_;
    std::vector<cv::Point2i> fps_;

    void initializeClassifier();
    void updateClassifier();
    void _compute();
    void _display() const;
    void _flush();
    void _reset();
    std::string _getName() const;
  };
  
} // namespace dst

#endif // PATCH_CLASSIFIER_H
