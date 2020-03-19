#include <online_learning/online_learning.h>

using namespace Eigen;
using namespace std;

#define NUM_CELLS (getenv("NUM_CELLS") ? atoi(getenv("NUM_CELLS")) : 10000)
#define SMOOTHING (getenv("SMOOTHING") ? atoi(getenv("SMOOTHING")) : 1)

namespace odontomachus
{
  
  /************************************************************
   * Cell
   ************************************************************/
  
  Cell::Cell(int num_classes, int smoothing) :
    counts_(smoothing * VectorXf::Ones(num_classes)),
    total_(smoothing * (num_classes + 1)),
    response_(VectorXf::Zero(num_classes))
  {
    ROS_ASSERT(smoothing > 0); // Otherwise you'll get divide-by-zeros.
    updateResponses();
  }

  void Cell::updateResponses()
  {
    //response_ = (2.0 * counts_ - VectorXf::Ones(counts_.rows()) * total_) / total_;
    for(int i = 0; i < response_.rows(); ++i)
      response_(i) = log(counts_(i) / (total_ - counts_(i)));
  }
  
  void Cell::train(int class_id)
  {
    ROS_ASSERT(class_id > -2);
    if(class_id >= 0)
      ++counts_(class_id);

    ++total_;
    updateResponses();
  }


  /************************************************************
   * WeakClassifier
   ************************************************************/
  
  WeakClassifier::WeakClassifier() :
    id_(-1),
    min_(FLT_MAX),
    max_(-FLT_MAX),
    width_(0),
    num_classes_(-1)
  {
  }
  
  void WeakClassifier::initialize(int id, int num_classes, const std::vector<Instance::Ptr>& data)
  {
    id_ = id;
    num_classes_ = num_classes;
    ROS_ASSERT(num_classes_ > 0);
    ROS_ASSERT(id_ >= 0);
    ROS_DEBUG_STREAM("Initializing weak classifier with id " << id_ << " using " << data.size() << " instances.");

    fallback_response_ = VectorXf::Zero(num_classes_);
   
    // -- Get the min and max.
    //    TODO: don't be so sensitive to outliers.  Fit a Gaussian, use 3stdevs or something.
    for(size_t i = 0;  i < data.size(); ++i) {
      const double& val = data[i]->descriptors_(id_);
      if(val > max_)
	max_ = val;
      if(val < min_)
	min_ = val;
    }
    ROS_DEBUG_STREAM("Max value during initialization: " << max_);
    ROS_DEBUG_STREAM("Min value during initialization: " << min_);
    
    // -- Choose a bin width.
    cells_.resize(NUM_CELLS);
    ROS_ASSERT(!cells_[0]);
    width_ = (max_ - min_) / (double)cells_.size();
    ROS_DEBUG_STREAM("Cell width set to " << width_);
    
    // -- Allocate cells.
    for(size_t i = 0; i < cells_.size(); ++i)
      cells_[i] = Cell::Ptr(new Cell(num_classes_, SMOOTHING));
  }
  
  void WeakClassifier::train(Instance::Ptr inst)
  {
    Cell::Ptr c = getCell(inst);
    if(c)
      c->train(inst->class_id_);
  }

  Cell::Ptr WeakClassifier::getCell(Instance::Ptr inst) const
  {
    double val = inst->descriptors_(id_);
    if(val < min_ || val >= max_) {
      Cell::Ptr nullptr((Cell*)NULL);
      ROS_ASSERT(!nullptr);
      return nullptr;
    }
    
    size_t idx = floor((val - min_) / width_);
    return cells_[idx];
  }
  
  const Eigen::VectorXf& WeakClassifier::classify(Instance::Ptr inst) const
  {
    Cell::Ptr c = getCell(inst);
    if(c)
      return getCell(inst)->response_;
    else
      return fallback_response_;
  }


  /************************************************************
   * Odontomachus
   ************************************************************/
  
  Odontomachus::Odontomachus() :
    num_classes_(-1)
  {
    ROS_DEBUG_STREAM("Using " << NUM_CELLS << " cells and smoothing of " << SMOOTHING);
  }
  
  void Odontomachus::initialize(int num_classes, const std::vector<Instance::Ptr>& data)
  {
    num_classes_ = num_classes;
    ROS_ASSERT(num_classes_ > 0);

    prior_ = VectorXf::Zero(num_classes_);
    counts_ = VectorXf::Ones(num_classes_) * SMOOTHING * NUM_CELLS;
    total_training_instances_ = SMOOTHING * NUM_CELLS * (num_classes_ + 1);
    updatePrior();
    
    // -- Make a new weak classifier for each 1D descriptor space.
    for(int i = 0; i < data[0]->descriptors_.rows(); ++i) {
      WeakClassifier::Ptr wc(new WeakClassifier());
      wc->initialize(i, num_classes_, data);
      wcs_.push_back(wc);
    }

  }
  
  void Odontomachus::train(Instance::Ptr inst)
  {
    ++total_training_instances_;
    if(inst->class_id_ >= 0)
      ++counts_(inst->class_id_);
    updatePrior();
    
    for(size_t i = 0; i < wcs_.size(); ++i)
      wcs_[i]->train(inst);
  }
  
  Classification Odontomachus::classify(Instance::Ptr inst) const
  {
    Classification cl;
    cl.response_ = (float)(wcs_.size() - 1) * (-prior_);
    for(size_t i = 0; i < wcs_.size(); ++i)
      cl.response_ += wcs_[i]->classify(inst);

    return cl;
  }

  void Odontomachus::updatePrior()
  {
    for(int i = 0; i < prior_.rows(); ++i)
      prior_(i) = log(counts_(i) / (total_training_instances_ - counts_(i)));
  }

} // namespace

