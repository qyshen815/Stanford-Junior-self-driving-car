#ifndef TRAINERS_H
#define TRAINERS_H

#include <online_learning/projection_slicer.h>
#include <online_learning/schedulers.h>

namespace odontomachus { 

  class Trainer
  {
  public:
    typedef boost::shared_ptr<Trainer> Ptr;
    typedef boost::shared_ptr<const Trainer> ConstPtr;

    virtual ~Trainer() {};
    virtual void attachSlicer(ProjectionSlicer::Ptr slicer);
    virtual void train(Dataset::ConstPtr dataset) = 0;
    
  protected:
    ProjectionSlicer::Ptr slicer_;    
  };
  
  class NaiveProjectionTrainer
  {
  public:
    typedef boost::shared_ptr<NaiveProjectionTrainer> Ptr;
    typedef boost::shared_ptr<const NaiveProjectionTrainer> ConstPtr;

    void setProjection(ProjectionSlicer::Ptr slicer, size_t idx);
    void applySmoothing(double smoothing);
    void increment(float val, int class_id);
    //! Fills projection_ with the appropriate values.
    void updateResponses() const;

  private:
    //! counts_(i, j) is the number of instances of class i seen in cell j.
    Eigen::MatrixXd counts_;
    //! totals_(i) is the total number of instances in cell i.
    Eigen::VectorXd totals_;
    ProjectionSlicer::Ptr slicer_;
    Projection::Ptr projection_;
  };
  
  class NaiveTrainer : public Trainer
  {
  public:
    typedef boost::shared_ptr<NaiveTrainer> Ptr;
    typedef boost::shared_ptr<const NaiveTrainer> ConstPtr;

    ~NaiveTrainer() {};
    NaiveTrainer();
    void setSmoothing(double smoothing);
    void attachSlicer(ProjectionSlicer::Ptr slicer);
    void train(Dataset::ConstPtr dataset);
    void setThreshold(Dataset::ConstPtr dataset);
    
  protected:
    double smoothing_;
    std::vector<NaiveProjectionTrainer::Ptr> projection_trainers_;
    Eigen::VectorXd counts_;
    double total_training_instances_;
    
    void updatePrior();
    void setThresholdForClass(int class_id,
			      const std::vector<Classification>& cls,
			      Dataset::ConstPtr dataset);
  };  
  
  class LogisticTrainer : public Trainer
  {
  public:
    typedef boost::shared_ptr<LogisticTrainer> Ptr;
    typedef boost::shared_ptr<const LogisticTrainer> ConstPtr;

    int max_num_iters_;
    
    LogisticTrainer();
    void train(Dataset::ConstPtr dataset);

  private:
    void trainForClass(int i, Dataset::ConstPtr dataset);
    void fillProblemData(int class_id,
			 Dataset::ConstPtr dataset,
			 Eigen::SparseMatrix<double, Eigen::ColMajor>* A,
			 Eigen::VectorXd* b) const;

  };

  class LogisticStochasticTrainer : public Trainer
  {
  public:
    typedef boost::shared_ptr<LogisticStochasticTrainer> Ptr;
    typedef boost::shared_ptr<const LogisticStochasticTrainer> ConstPtr;
    
    //! num_instances_seen_(i) is number seen of class i.
    Eigen::VectorXi num_instances_seen_;
    //! Must be set by the user.  Is deleted by LogisticStochasticTrainer.
    LearningRateScheduler* scheduler_;

    LogisticStochasticTrainer();
    ~LogisticStochasticTrainer();
    void attachSlicer(ProjectionSlicer::Ptr slicer);
    //! Takes a gradient step for each training instance in the dataset.
    //! Uses a random order.
    void train(Dataset::ConstPtr dataset);

  private:
    void step(const Eigen::VectorXf& descriptor, double response, double y, int response_class_id);
  };

  class HybridTrainer : public NaiveTrainer
  {
  public:
    HybridTrainer();
    //! Runs logistic regression for each class problem.
    void trainProjectionWeights(Dataset::ConstPtr dataset);

  private:
    void trainProjectionWeightsForClass(int class_id, Dataset::ConstPtr dataset);
  };
  
  class HybridStochasticTrainer : public NaiveTrainer
  {
  public:
    typedef boost::shared_ptr<HybridStochasticTrainer> Ptr;
    typedef boost::shared_ptr<const HybridStochasticTrainer> ConstPtr;

    //! Must be set by the user.  Is deleted by HybridStochasticTrainer.
    LearningRateScheduler* scheduler_;
    
    HybridStochasticTrainer();
    ~HybridStochasticTrainer();
    //! Takes a gradient step for each training instance in the dataset.
    void trainProjectionWeights(Dataset::ConstPtr dataset);

  private:
    void step(const Eigen::VectorXf& descriptor, double response, double y, int response_class_id);
  };

} // namespace odontomachus

#endif // TRAINERS_H
