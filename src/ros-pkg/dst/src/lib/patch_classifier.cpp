#include <dst/patch_classifier.h>

using namespace std;
using namespace Eigen;
using boost::shared_ptr;

namespace dst
{

  PatchClassifierData::PatchClassifierData()
  {
  }
  
  PatchClassifierData::PatchClassifierData(cv::Mat3b bgr, cv::Mat1b intensity,
					   cv::Mat1i integral, cv::Mat3f hsv) :
    bgr_(bgr),
    intensity_(intensity),
    integral_(integral),
    hsv_(hsv)
  {
  }
  
  /************************************************************
   * Features
   ************************************************************/

  BinaryFeature::BinaryFeature(int radius) :
    radius_(radius),
    window_width_(2 * radius_ + 1)
  {
  }

  cv::Point2i BinaryFeature::getRandomPoint() const
  {
    cv::Point2i pt;
    pt.x = (rand() % window_width_) - radius_;
    pt.y = (rand() % window_width_) - radius_;
    return pt;
  }

  std::string BinaryFeature::status() const
  {
    ostringstream oss;
    oss << "BinaryFeature" << endl;
    oss << "Radius: " << radius_ << endl;
    oss << "Width: " << window_width_ << endl;
    return oss.str();
  }
  
  PointColorThreshold::PointColorThreshold(int radius) :
    BinaryFeature(radius)
  {
    pt_ = getRandomPoint();
    channel_ = rand() % 3;
    sign_ = 2 * (rand() % 2) - 1;
    threshold_ = sign_ * rand() % 256;
  }

  bool PointColorThreshold::compute(PatchClassifierData& data, const cv::Point2i& pt) const
  {
    return sign_ * data.bgr_(pt + pt_)[channel_] < threshold_;
  }

  string PointColorThreshold::status() const
  {
    ostringstream oss;
    oss << "PointColorThreshold" << endl;
    oss << "Point: " << pt_ << endl;
    oss << "Channel: " << channel_ << endl;
    oss << "Threshold: " << threshold_ << endl;
    oss << "Sign: " << sign_ << endl;
    oss << BinaryFeature::status();
    return oss.str();
  }

  IntensityDifference::IntensityDifference(int radius) :
    BinaryFeature(radius)
  {
    pt0_ = getRandomPoint();
    pt1_ = getRandomPoint();
  }

  bool IntensityDifference::compute(PatchClassifierData& data, const cv::Point2i& pt) const
  {
    return data.intensity_(pt + pt0_) < data.intensity_(pt + pt1_);
  }

  std::string IntensityDifference::status() const
  {
    ostringstream oss;
    oss << "IntensityDifference" << endl;
    oss << "Point0: " << pt0_ << endl;
    oss << "Point1: " << pt1_ << endl;
    oss << BinaryFeature::status();
    return oss.str();
  }

  ColorDifference::ColorDifference(int radius) :
    BinaryFeature(radius)
  {
    pt0_ = getRandomPoint();
    pt1_ = getRandomPoint();
    channel0_ = rand() % 3;
    channel1_ = rand() % 3;
  }
  
  bool ColorDifference::compute(PatchClassifierData& data, const cv::Point2i& pt) const
  {
    return (data.bgr_(pt + pt0_)[channel0_] < data.bgr_(pt + pt1_)[channel1_]);
  }
  
  std::string ColorDifference::status() const
  {
    ostringstream oss;
    oss << "ColorDifference" << endl;
    oss << "Point0: " << pt0_ << endl;
    oss << "Point1: " << pt1_ << endl;
    oss << "Channel0: " << channel0_ << endl;
    oss << "Channel1: " << channel1_ << endl;
    oss << BinaryFeature::status();
    return oss.str();
  }

  HaarWavelet::HaarWavelet(int radius) :
    BinaryFeature(radius)
  {
    ROS_ASSERT(radius_ != 0);
    while(true) { 
      orientation_ = rand() % 2;
      sign_ = 2 * (rand() % 2) - 1;
      width_ = (rand() % radius_) + 1;
      height_ = (rand() % radius_) + 1;
      ul0_ = getRandomPoint();
      if(orientation_ == 0) {
	ul1_.x = ul0_.x + width_;
	ul1_.y = ul0_.y;
      }
      else {
	ul1_.x = ul0_.x;
	ul1_.y = ul0_.y + height_;
      }
      if(ul1_.x + width_ < radius_ && ul1_.y + height_ < radius_)
	break;
    }
  }

  int HaarWavelet::evaluateBox(cv::Mat1i integral, const cv::Point2i ul, int width, int height) const
  {
    cv::Point2i lr = ul + cv::Point2i(width, height);
    cv::Point2i ll = ul + cv::Point2i(0, height);
    cv::Point2i ur = ul + cv::Point2i(width, 0);
    return integral(lr) + integral(ul) - integral(ll) - integral(ur);
  }
  
  bool HaarWavelet::compute(PatchClassifierData& data, const cv::Point2i& pt) const
  {
    return sign_ * evaluateBox(data.integral_, pt + ul0_, width_, height_) > sign_ * evaluateBox(data.integral_, pt + ul1_, width_, height_);
  }

  std::string HaarWavelet::status() const
  {
    ostringstream oss;
    oss << "HaarWavelet" << endl;
    oss << "Orientation: " << orientation_ << endl;
    oss << "Sign: " << sign_ << endl;
    oss << "Width: " << width_ << endl;
    oss << "Height: " << height_ << endl;
    oss << "UL0: " << ul0_ << endl;
    oss << "UL1: " << ul1_ << endl;
    oss << BinaryFeature::status();
    return oss.str();
  }

    
  /************************************************************
   * Fern
   ************************************************************/
  
  Fern::Fern(int radius) :
    radius_(radius),
    width_(2 * radius_ + 1)
  {
    positive_.setOnes(); // Laplace smoothing.
    negative_.setOnes();
    total_.setZero();
    output_.setZero();
    
    initialize();
  }

  Fern::~Fern()
  {
  }

  void Fern::initialize()
  {
    features_.reserve(NUM_FEATURES);
    for(int i = 0; i < NUM_FEATURES; ++i) { 
      int type = rand() % 7;
      if(type == 0 || type == 1)
      	features_.push_back(ColorDifference::Ptr(new ColorDifference(radius_)));
      else if(type == 2 || type == 3)
      	features_.push_back(IntensityDifference::Ptr(new IntensityDifference(radius_)));
      else if(type == 4)
       	features_.push_back(PointColorThreshold::Ptr(new PointColorThreshold(radius_)));
      else if(type == 5 || type == 6)
	features_.push_back(HaarWavelet::Ptr(new HaarWavelet(radius_)));
    }
  }
  
  void Fern::train(PatchClassifierData& data, const cv::Point2i& pt, uchar label)
  {
    FeatureVector f = computeFeatures(data, pt);
    train(f, label);
  }

  void Fern::train(const FeatureVector& f, uchar label)
  {
    if(label == 255) {
      ++positive_(f);
      //++total_(f);
      //output_(f) = sign(positive_(f) - negative_(f));
      //output_(f) = (positive_(f) - negative_(f)) / total_(f);
    }
    else if(label == 0) { 
      ++negative_(f);
      //++total_(f);
      //output_(f) = sign(positive_(f) - negative_(f));
      //output_(f) = (positive_(f) - negative_(f)) / total_(f);
    }
  }

  inline
  float Fern::classify(PatchClassifierData& data, const cv::Point2i& pt) const
  {
    FeatureVector f = computeFeatures(data, pt);
    return log(positive_(f) / negative_(f));
    //return output_(f);
  }

  inline
  float Fern::classify(const FeatureVector& f) const
  {
    //return output_(f);
    return log(positive_(f) / negative_(f));
  }

  Fern::FeatureVector Fern::computeFeatures(PatchClassifierData& data, const cv::Point2i& pt) const
  {
    FeatureVector features = 0;
    int bit = 1;
    for(int i = 0; i < NUM_FEATURES; ++i) {
      if(features_[i]->compute(data, pt))
	features |= bit;
      bit *= 2;
    }
    return features;
  }

  std::string Fern::status() const
  {
    ostringstream oss;
    oss << "Radius: " << radius_ << endl;
    for(int i = 0; i < NUM_FEATURES; ++i) {
      oss << features_[i]->status();
      oss << "--------------------" << endl;
    }
    oss << "  Positive counts: " << endl << positive_.transpose() << endl;
    oss << "  Negative counts: " << endl << negative_.transpose() << endl;
    oss << "  Output: " << endl << output_.transpose() << endl;
    oss << "  Total positive seen: " << positive_.sum() << endl;
    oss << "  Total negative seen: " << negative_.sum() << endl;

    return oss.str();
  }
  
  /************************************************************
   * PatchClassifier
   ************************************************************/

  PatchClassifier::PatchClassifier(int radius) :
    radius_(radius),
    total_pos_(0),
    total_neg_(0),
    offset_(0)
  {
    ferns_.resize(NUM_FERNS);
    for(int i = 0; i < NUM_FERNS; ++i) {
      ferns_[i] = Fern::Ptr(new Fern(radius_));
    }
  }

  void PatchClassifier::train(PatchClassifierData& data, const cv::Point2i& pt, uchar label)
  {
    if(label == 255)
      ++total_pos_;
    else if(label == 0)
      ++total_neg_;
    else
      return;
        
    for(size_t i = 0; i < ferns_.size(); ++i)
      ferns_[i]->train(data, pt, label);
  }

  void PatchClassifier::train(FeatureCache& cache, const cv::Point2i& pt, uchar label)
  {
    if(label == 255)
      ++total_pos_;
    else if(label == 0)
      ++total_neg_;
    else
      return;
    
    for(size_t i = 0; i < ferns_.size(); ++i)
      ferns_[i]->train(cache(pt)[i], label);
  }

  void PatchClassifier::setOffsetByPrior()
  {
    cout << "total_pos_ " << total_pos_ << endl;
    cout << "total_neg_ " << total_neg_ << endl;
    offset_ = (1.0 - (float)ferns_.size()) * log(total_pos_ / total_neg_) / (float)ferns_.size();
    cout << "Offset set to " << offset_ << endl;
  }
  
  void PatchClassifier::learnOffset(cv::Mat1f cached_classifications, cv::Mat1b labels)
  {
    ROS_ASSERT(offset_ == 0);
    
    int num_vars = (labels.rows - 2 * radius_) * (labels.cols - 2 * radius_);
    shared_ptr<MatrixXd> A(new MatrixXd(1, num_vars));
    shared_ptr<VectorXd> b(new VectorXd(num_vars));

    double min_prediction = std::numeric_limits<double>::max();
    double max_prediction = -std::numeric_limits<double>::max();
    int idx = 0;
    for(int y = radius_; y < labels.rows - radius_; ++y) {
      for(int x = radius_; x < labels.cols - radius_; ++x, ++idx) {
	float prediction = cached_classifications(y, x);
	if(prediction > max_prediction)
	  max_prediction = prediction;
	if(prediction < min_prediction)
	  min_prediction = prediction;
	
	if(labels(y, x) == 0) {
	  A->coeffRef(idx) = 1;
	  b->coeffRef(idx) = cached_classifications(y, x);
	}
	else if(labels(y, x) == 255) {
	  A->coeffRef(idx) = -1;
	  b->coeffRef(idx) = -cached_classifications(y, x);
	}
	else {
	  A->coeffRef(idx) = 0;
	  b->coeffRef(idx) = 0;
	}
      }
    }
    ROS_ASSERT(idx == num_vars);

    ObjectiveMLSNoCopy obj(A, b);
    GradientMLSNoCopy grad(A, b);
    BisectionSolver bi(&obj, &grad, 1e-6, min_prediction, max_prediction, 0, true);
    offset_ = bi.solve();
  }
  
  float PatchClassifier::classify(PatchClassifierData& data, const cv::Point2i& pt) const
  {
    float val = 0.0;
    for(size_t i = 0; i < ferns_.size(); ++i) {
      val += ferns_[i]->classify(data, pt);
    }
    val /= (float)ferns_.size();
    val += offset_;
     
    return val;
  }

  float PatchClassifier::classify(FeatureCache& cache,
				  const cv::Point2i& pt) const
  {
    float val = 0.0;
    for(int i = 0; i < NUM_FERNS; ++i) { 
      val += ferns_[i]->classify(cache(pt)[i]);
    }
    val /= (float)ferns_.size();
    val += offset_;

    return val;
  }

  void PatchClassifier::cacheFeatures(PatchClassifierData& data,
				      FeatureCache& cache) const
  {
    cv::Point2i pt;
    cv::Mat3b bgr = data.bgr_;
    for(pt.y = radius_; pt.y < bgr.rows - radius_; ++pt.y) 
      for(pt.x = radius_; pt.x < bgr.cols - radius_; ++pt.x)
	for(int i = 0; i < NUM_FERNS; ++i)
	  cache(pt)[i] = ferns_[i]->computeFeatures(data, pt);
  }

  std::string PatchClassifier::status() const
  {
    ostringstream oss;
    for(size_t i = 0; i < ferns_.size(); ++i) {
      oss << "====================" << endl;
      oss << "Fern " << i << endl;
      oss << ferns_[i]->status() << endl;
      oss << "====================" << endl;
    }
    return oss.str();
  }
  
  /************************************************************
   * PatchClassifierNPG
   ************************************************************/

  PatchClassifierNPG::PatchClassifierNPG(pipeline2::Outlet<cv::Mat3b>* img_otl,
					 pipeline2::Outlet<cv::Mat1b>* intensity_otl,
					 pipeline2::Outlet<cv::Mat1i>* integral_otl,
					 pipeline2::Outlet<cv::Mat3f>* hsv_otl,
					 pipeline2::Outlet<cv::Mat1b>* prev_seg_otl,
					 int radius) :
    NodePotentialGenerator(),
    img_otl_(img_otl),
    intensity_otl_(intensity_otl),
    integral_otl_(integral_otl),
    hsv_otl_(hsv_otl),
    prev_seg_otl_(prev_seg_otl),
    radius_(radius),
    classifier_(new PatchClassifier(radius_))
  {
    registerInput(img_otl_->getNode());
    registerInput(intensity_otl_->getNode());
    registerInput(integral_otl_->getNode());
    registerInput(hsv_otl_->getNode());
    registerInput(prev_seg_otl_->getNode());

  }

  void PatchClassifierNPG::initializeClassifier()
  {
    cv::Mat1b prev_seg = prev_seg_otl_->pull();
    cached_classifications_ = cv::Mat1f(prev_seg.size(), 0);
    cached_features_ = PatchClassifier::FeatureCache(prev_seg.size());
    classifier_->cacheFeatures(prev_data_, cached_features_);
    fps_.reserve(prev_seg.rows * prev_seg.cols);
  }

  void PatchClassifierNPG::updateClassifier()
  {
    // -- Train on all positive examples in the previous image.
    cv::Mat3b prev_bgr = prev_data_.bgr_;
    cv::Mat1b prev_seg = prev_seg_otl_->pull();
    cv::Point2i pt;
    int num_pos = 0;
    for(pt.y = radius_; pt.y < prev_bgr.rows - radius_; ++pt.y) {
      for(pt.x = radius_; pt.x < prev_bgr.cols - radius_; ++pt.x) {
	if(prev_seg(pt) == 255) {
	  classifier_->train(cached_features_, pt, 255);
	  ++num_pos;
	}
      }
    }
    
    // -- Train on num_pos randomly selected false positives
    //    from the previous image.
    fps_.clear();
    for(pt.y = radius_; pt.y < prev_bgr.rows - radius_; ++pt.y)
      for(pt.x = radius_; pt.x < prev_bgr.cols - radius_; ++pt.x) 
	if(prev_seg(pt) == 0 && classifier_->classify(cached_features_, pt) > 0) 
	  fps_.push_back(pt);
    
    std::random_shuffle(fps_.begin(), fps_.end());
    for(int i = 0; i < num_pos; ++i)
      classifier_->train(cached_features_, fps_[i], 0);
    
  }
  
  void PatchClassifierNPG::_compute()
  { 
    prev_data_ = data_;
    data_.bgr_ = img_otl_->pull();
    data_.intensity_ = intensity_otl_->pull();
    data_.integral_ = integral_otl_->pull();
    data_.hsv_ = hsv_otl_->pull();

    cv::Mat1b prev_seg = prev_seg_otl_->pull();
    if(prev_seg.rows == 0) {
      source_otl_.push(NULL);
      sink_otl_.push(NULL);
      return;
    }

    if(source_potentials_.rows() != prev_seg.rows ||
       source_potentials_.cols() != prev_seg.cols) {
      source_potentials_ = MatrixXd::Zero(prev_seg.rows, prev_seg.cols);
      sink_potentials_ = MatrixXd::Zero(prev_seg.rows, prev_seg.cols);
    }

    if(cached_classifications_.rows != prev_seg.rows)
      initializeClassifier();

    updateClassifier();    
    
    // -- Classify the current image.
    classifier_->cacheFeatures(data_, cached_features_);
    cv::Mat3b bgr = data_.bgr_;
    cv::Point2i pt;
    for(pt.y = radius_; pt.y < bgr.rows - radius_; ++pt.y) { 
      for(pt.x = radius_; pt.x < bgr.cols - radius_; ++pt.x) { 
	float logodds = classifier_->classify(cached_features_, pt);
	cached_classifications_(pt) = logodds;
	float prfg = 1.0 / (1.0 + exp(-logodds));
	float prbg = 1.0 - prfg;
	source_potentials_(pt.y, pt.x) = prfg;
	sink_potentials_(pt.y, pt.x) = prbg;
      }
    }
        
    // -- Fill outlets.
    source_otl_.push(&source_potentials_);
    sink_otl_.push(&sink_potentials_);
  }

  void PatchClassifierNPG::_display() const
  {
    //cout << classifier_->status() << endl;
    cv::Mat1b prev_seg = prev_seg_otl_->pull();
    if(prev_seg.rows == 0)
      return;

    displayNodePotentials(img_otl_->pull());
  }

  void PatchClassifierNPG::_flush()
  {
    source_potentials_.setZero();
    sink_potentials_.setZero();
    source_otl_.push(NULL);
    sink_otl_.push(NULL);
  }

  void PatchClassifierNPG::_reset()
  {
    classifier_ = PatchClassifier::Ptr(new PatchClassifier(radius_));
    data_ = PatchClassifierData();
    prev_data_ = PatchClassifierData();
    cached_classifications_ = cv::Mat1f();
  }

  std::string PatchClassifierNPG::_getName() const
  {
    std::ostringstream oss;
    oss << "PatchClassifierNPG_radius:" << radius_;
    return oss.str();
  }
  
} // namespace dst
