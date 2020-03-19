#ifndef PROJECTION_SLICER_H
#define PROJECTION_SLICER_H

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Sparse>
#include <unconstrained_optimization/unconstrained_optimization.h>
#include <unconstrained_optimization/common_functions.h>
#include <logistic/logistic.h>
#include <bag_of_tricks/high_res_timer.h>
#include <online_learning/online_learning.h>
#include <performance_statistics/performance_statistics.h>
#include <float.h>

namespace odontomachus { 
  
  class Projection : public Serializable
  {
  public:
    typedef boost::shared_ptr<Projection> Ptr;
    typedef boost::shared_ptr<const Projection> ConstPtr;
    static const int NO_CELL = -1;

    //! cells_(i, j) is the response for class i, cell j.
    Eigen::MatrixXf cells_;
    Eigen::VectorXf fallback_response_;
    
    Projection();
    //! Sets the bounds and allocates bins based on a sample of data.
    void initialize(size_t num_classes, size_t num_cells, const Eigen::VectorXf& vals);
    Eigen::VectorXf classify(float val) const;
    int getCellIdx(float val) const;
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    
  private:
    double min_;
    double max_;
    double width_;
  };

  class ProjectionSlicer : public Serializable
  {
  public:
    typedef boost::shared_ptr<ProjectionSlicer> Ptr;
    typedef boost::shared_ptr<const ProjectionSlicer> ConstPtr;
     
    //! projection_weights_(i, j) is the weight for class i, projection j.
    Eigen::MatrixXf projection_weights_;
    std::vector<Projection::Ptr> projections_;
    Eigen::VectorXf prior_;
    //! Tunable threshold.
    Eigen::VectorXf offset_;
    
    ProjectionSlicer();
    //! Initializes all weak classifiers.  Doesn't need to be labeled.
    void initialize(size_t num_classes, size_t num_cells, Dataset::ConstPtr dataset);
    Classification classify(const Eigen::VectorXf& instance) const;
    size_t getNumCells() const;
    size_t getNumClasses() const;
    //! Returns numbers of cells in all projections, plus one for the prior.
    //! Does not include the projection_weights_.
    size_t getNumParamsPerClass() const;

    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
  };

  class Evaluator
  {
  public:
    Evaluator(ProjectionSlicer::Ptr slicer);
    void evaluate(Dataset::ConstPtr test);
    void saveResults(const std::string& path);
    
  private:
    ProjectionSlicer::Ptr slicer_;
    PerfStats frame_stats_;
    PerfStats track_stats_;
  };
  
} // namespace

#endif // PROJECTION_SLICER_H
