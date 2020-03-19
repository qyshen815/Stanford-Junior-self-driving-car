#include <online_learning/projection_slicer.h>

using namespace std;
using namespace Eigen;
using boost::shared_ptr;

namespace odontomachus
{

  /************************************************************
   * Projection
   ************************************************************/
  
  Projection::Projection() :
    min_(FLT_MAX),
    max_(-FLT_MAX),
    width_(0)
  {
    
  }

  void Projection::initialize(size_t num_classes, size_t num_cells, const Eigen::VectorXf& vals)
  {
    bool verbose = false;
    if(rand() % 100 == 0)
      verbose = true;

    ROS_ASSERT(num_classes > 0);
    if(verbose)
      ROS_DEBUG_STREAM("Initializing Projection using " << vals.rows() << " instances.");

    fallback_response_ = VectorXf::Zero(num_classes);
   
    // -- Get the min and max.
    //    TODO: don't be so sensitive to outliers.  Fit a Gaussian, use 3stdevs or something.
    max_ = vals.maxCoeff();
    min_ = vals.minCoeff();
    if(verbose) { 
      ROS_DEBUG_STREAM("Max value during initialization: " << max_);
      ROS_DEBUG_STREAM("Min value during initialization: " << min_);
    }
    
    // -- Choose a bin width.
    width_ = (max_ - min_) / (double)num_cells;
    if(verbose)
      ROS_DEBUG_STREAM("Cell width set to " << width_);
    
    // -- Allocate cells.
    cells_ = MatrixXf::Zero(num_classes, num_cells);
  }

  int Projection::getCellIdx(float val) const
  {
    if(val < min_ || val >= max_)
      return NO_CELL;

    int idx = floor((val - min_) / width_);
    ROS_ASSERT(idx >= 0 && idx < cells_.cols());
    return idx;
  }
  
  Eigen::VectorXf Projection::classify(float val) const
  {
    int idx = getCellIdx(val);
    if(idx == NO_CELL)
      return fallback_response_;
    else
      return cells_.col(idx);
  }

  void Projection::serialize(std::ostream& out) const
  {
    eigen_extensions::serialize(cells_, out);
    eigen_extensions::serialize(fallback_response_, out);
    out.write((char*)&min_, sizeof(min_));
    out.write((char*)&max_, sizeof(max_));
    out.write((char*)&width_, sizeof(width_));
  }
  
  void Projection::deserialize(std::istream& in)
  {
    eigen_extensions::deserialize(in, &cells_);
    eigen_extensions::deserialize(in, &fallback_response_);
    in.read((char*)&min_, sizeof(min_));
    in.read((char*)&max_, sizeof(max_));
    in.read((char*)&width_, sizeof(width_));
  }
  
  /************************************************************
   * ProjectionSlicer
   ************************************************************/
  
  ProjectionSlicer::ProjectionSlicer()
  {
  }

  void ProjectionSlicer::initialize(size_t num_classes, size_t num_cells, Dataset::ConstPtr dataset)
  {
    ROS_ASSERT(num_classes > 0);
    prior_ = VectorXf::Zero(num_classes);
    
    projections_.reserve(dataset->descriptors_.rows());
    for(int i = 0; i < dataset->descriptors_.rows(); ++i)
      projections_.push_back(Projection::Ptr(new Projection()));

    for(size_t i = 0; i < projections_.size(); ++i)
      projections_[i]->initialize(num_classes, num_cells, dataset->descriptors_.row(i));
    
    projection_weights_ = MatrixXf::Ones(num_classes, projections_.size());
    offset_ = VectorXf::Zero(num_classes);
  }

  Classification ProjectionSlicer::classify(const Eigen::VectorXf& instance) const
  {
    ROS_ASSERT((size_t)instance.rows() == projections_.size());
    
    Classification cl;
    cl.response_ = prior_ + offset_;
    for(size_t i = 0; i < projections_.size(); ++i)
      cl.response_ += projection_weights_.col(i).cwiseProduct(projections_[i]->classify(instance(i)));

    return cl;
  }

  size_t ProjectionSlicer::getNumCells() const
  {
    ROS_ASSERT(!projections_.empty());
    return projections_[0]->cells_.cols();
  }

  size_t ProjectionSlicer::getNumClasses() const
  {
    return prior_.rows();
  }

  size_t ProjectionSlicer::getNumParamsPerClass() const
  {
    int num_params = 1; // prior
    for(size_t i = 0; i < projections_.size(); ++i)
      num_params += projections_[i]->cells_.cols();
    return num_params;
  }
  
  void ProjectionSlicer::serialize(std::ostream& out) const
  {
    eigen_extensions::serialize(projection_weights_, out);
    ROS_ASSERT((size_t)projection_weights_.cols() == projections_.size());
    for(size_t i = 0; i < projections_.size(); ++i)
      projections_[i]->serialize(out);
    eigen_extensions::serialize(prior_, out);
    eigen_extensions::serialize(offset_, out);
  }
  
  void ProjectionSlicer::deserialize(std::istream& in)
  {
    eigen_extensions::deserialize(in, &projection_weights_);
    for(int i = 0; i < projection_weights_.cols(); ++i) {
      Projection::Ptr proj(new Projection());
      proj->deserialize(in);
      projections_.push_back(proj);
    }
    eigen_extensions::deserialize(in, &prior_);
    eigen_extensions::deserialize(in, &offset_);
  }  
  
} // namespace
