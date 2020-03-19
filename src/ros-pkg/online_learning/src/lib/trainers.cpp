#include <online_learning/trainers.h>

using namespace std;
using namespace Eigen;
using boost::shared_ptr;

namespace odontomachus
{

  void Trainer::attachSlicer(ProjectionSlicer::Ptr slicer)
  {
    ROS_ASSERT(!slicer_);
    ROS_ASSERT(slicer);
    ROS_ASSERT(slicer->prior_.rows() > 0);
    ROS_FATAL_STREAM_COND(slicer->projections_.empty(), "Attempted to attach uninitialized projection slicer to trainer.");
    slicer_ = slicer;
  }
  
/************************************************************
 * NaiveProjectionTrainer
 ************************************************************/

  void NaiveProjectionTrainer::setProjection(ProjectionSlicer::Ptr slicer, size_t idx)
  {
    ROS_ASSERT(!slicer_);
    
    slicer_ = slicer;
    projection_ = slicer_->projections_[idx];
    counts_ = MatrixXd::Zero(projection_->cells_.rows(), projection_->cells_.cols());
    totals_ = VectorXd::Zero(counts_.cols());
  }

  void NaiveProjectionTrainer::applySmoothing(double smoothing)
  {
    ROS_ASSERT(slicer_);
    
    counts_.setOnes() * smoothing;
    totals_ = VectorXd::Ones(counts_.cols()) * smoothing * (counts_.rows() + 1.0);
  }

  void NaiveProjectionTrainer::increment(float val, int class_id)
  {
    ROS_ASSERT(slicer_);

    if(class_id < -1)
      return;
    
    int idx = projection_->getCellIdx(val);
    if(idx == Projection::NO_CELL)
      return;
    
    ++totals_(idx);
    if(class_id >= 0)
      ++counts_(class_id, idx);
  }
  
  void NaiveProjectionTrainer::updateResponses() const
  {
    ROS_ASSERT(slicer_);
    
    for(int i = 0; i < projection_->cells_.cols(); ++i) {
      for(int j = 0; j < projection_->cells_.rows(); ++j) {
	projection_->cells_(j, i) = -slicer_->prior_(j) + log(counts_(j, i) / (totals_(i) - counts_(j, i)));
      }
    }
  }


  /************************************************************
   * NaiveTrainer
   ************************************************************/

  NaiveTrainer::NaiveTrainer() :
    Trainer(),
    smoothing_(0),
    total_training_instances_(0)
  {
    ROS_ASSERT(!slicer_);
  }
  
  void NaiveTrainer::setSmoothing(double smoothing)
  {
    ROS_ASSERT(!slicer_);
    smoothing_ = smoothing;
  }
  
  void NaiveTrainer::attachSlicer(ProjectionSlicer::Ptr slicer)
  {
    Trainer::attachSlicer(slicer);
    ROS_FATAL_STREAM_COND(smoothing_ <= 0, "NaiveTrainer must have positive smoothing; it is currently set to " << smoothing_);
    
    // -- Initialize projection trainers.
    projection_trainers_.resize(slicer_->projections_.size());
    for(size_t i = 0; i < projection_trainers_.size(); ++i) {
      projection_trainers_[i] = NaiveProjectionTrainer::Ptr(new NaiveProjectionTrainer());
      projection_trainers_[i]->setProjection(slicer_, i);
      projection_trainers_[i]->applySmoothing(smoothing_);
    }

    // -- Set initial global counts.
    counts_ = VectorXd::Ones(slicer_->getNumClasses()) * smoothing_ * slicer_->getNumCells();
    total_training_instances_ = smoothing_ * slicer_->getNumCells() * (slicer_->getNumClasses() + 1);
  }
  
  void NaiveTrainer::train(Dataset::ConstPtr dataset)
  {
    ROS_ASSERT(slicer_);
    
    // -- Update global counts and the prior.
    for(int i = 0; i < dataset->labels_.rows(); ++i) {
      if(dataset->labels_(i) == -2)
	continue;

      ++total_training_instances_;
      if(dataset->labels_(i) != -1)
	++counts_(dataset->labels_(i));
    }
    updatePrior();

    // -- Accumulate counts in the projections.
    ROS_ASSERT((size_t)dataset->descriptors_.rows() == projection_trainers_.size());
    for(int i = 0; i < dataset->descriptors_.cols(); ++i)
      for(size_t j = 0; j < projection_trainers_.size(); ++j)
	projection_trainers_[j]->increment(dataset->descriptors_(j, i), dataset->labels_(i));

    // -- Update response values.
    for(size_t i = 0; i < projection_trainers_.size(); ++i)
      projection_trainers_[i]->updateResponses();
  }

  void NaiveTrainer::updatePrior()
  {
    for (int i = 0; i < slicer_->prior_.rows(); ++i)
      slicer_->prior_(i) = log(counts_(i) / (total_training_instances_ - counts_(i)));
  }

  void NaiveTrainer::setThreshold(Dataset::ConstPtr dataset)
  {
    ROS_DEBUG_STREAM("Setting threshold." << flush);
    slicer_->offset_ = VectorXf::Zero(slicer_->prior_.rows());
    
    // -- Get classifications for everything in the dataset.
    vector<Classification> cls(dataset->labels_.rows());
    for (int i = 0; i < dataset->labels_.rows(); ++i) {
      if (dataset->labels_(i) == -2)
	continue;
      
      cls[i] = slicer_->classify(dataset->descriptors_.col(i));
    }

    for (size_t i = 0; i < slicer_->getNumClasses(); ++i)
      setThresholdForClass(i, cls, dataset);
  }

  void NaiveTrainer::setThresholdForClass(int class_id,
					  const vector<Classification>& cls,
					  Dataset::ConstPtr dataset)
  {
    // -- Set up data matrices.
    shared_ptr<MatrixXd> A(new MatrixXd(1, dataset->labels_.rows()));
    for(int i = 0; i < dataset->labels_.rows(); ++i) {
      if(dataset->labels_(i) == -2)
	A->coeffRef(0, i) = 0.0;
      else if(dataset->labels_(i) == class_id)
	A->coeffRef(0, i) = -1.0;
      else
	A->coeffRef(0, i) = 1.0;
    }

    shared_ptr<VectorXd> b(new VectorXd());
    *b = VectorXd::Zero(dataset->labels_.rows());
    for(int i = 0; i < dataset->labels_.rows(); ++i) {
      if(dataset->labels_(i) == -2)
	b->coeffRef(i) = 0;
      else if(dataset->labels_(i) == class_id)
	b->coeffRef(i) = -cls[i].response_(class_id);
      else
	b->coeffRef(i) = cls[i].response_(class_id);
    }
    
    // -- Use logistic regression solver.
    Logistic lg;
    lg.A_ = A;
    lg.b_ = b;
    lg.debug_ = false;
    lg.max_num_iters_ = 1000;
    lg.train();
    slicer_->offset_(class_id) = lg.getIntercept();
    ROS_DEBUG_STREAM("Set threshold for class " << class_id << " to " << lg.getIntercept() << flush);
  }

  
  /************************************************************
   * LogisticTrainer
   ************************************************************/

  LogisticTrainer::LogisticTrainer() :
    max_num_iters_(0)
  {
  }
  
  void LogisticTrainer::train(Dataset::ConstPtr dataset)
  {
    ROS_ASSERT(slicer_);
    
    for(size_t i = 0; i < slicer_->getNumClasses(); ++i) { 
      trainForClass(i, dataset);
    }
  }

  void LogisticTrainer::trainForClass(int class_id, Dataset::ConstPtr dataset)
  {
    Eigen::SparseMatrix<double, Eigen::ColMajor> A;
    VectorXd b;
    fillProblemData(class_id, dataset, &A, &b);
    
    ObjectiveMLSSparse objective(&A, &b);
    GradientMLSSparse gradient(&A, &b);

    double tol = 1e-6;
    double alpha = 0.4;
    double beta = 0.5;
    double initial_stepsize = 1.0;
    bool debug = true;

    VectorXd init = VectorXd::Zero(slicer_->getNumParamsPerClass());
    VectorXd params;
    if(getenv("USE_GRADIENT")) {
      cout << "Using gradient" << endl;
      GradientSolver solver(&objective, &gradient, tol, alpha, beta, max_num_iters_, initial_stepsize, debug);
      HighResTimer hrt("Gradient");
      hrt.start();
      params = solver.solve(init);
      hrt.stop();
      cout << hrt.reportSeconds() << endl;
    }
    else {
      cout << "Using Nesterov." << endl;
      NesterovGradientSolver solver(&objective, &gradient, tol, alpha, beta, max_num_iters_, initial_stepsize, debug);
      HighResTimer hrt("Nesterov");
      hrt.start();
      params = solver.solve(init);
      hrt.stop();
      cout << hrt.reportSeconds() << endl;
    }
    
    ROS_DEBUG_STREAM("Learned params: " << params.transpose() << flush);
    
    // -- Install params.
    slicer_->prior_(class_id) = params(0);
    
    int idx = 1;
    for(size_t i = 0; i < slicer_->projections_.size(); ++i) {
      for(int j = 0; j < slicer_->projections_[i]->cells_.cols(); ++j, ++idx) { 
	slicer_->projections_[i]->cells_(class_id, j) = params(idx);
      }
    }
  }

  void LogisticTrainer::fillProblemData(int class_id,
					Dataset::ConstPtr dataset,
					Eigen::SparseMatrix<double, Eigen::ColMajor>* A,
					Eigen::VectorXd* b) const
  {
    *b = VectorXd::Zero(dataset->descriptors_.cols());
    *A = Eigen::SparseMatrix<double, Eigen::ColMajor>(slicer_->getNumParamsPerClass(), dataset->descriptors_.cols());

    int expected_num_nonzero = dataset->descriptors_.cols() * (slicer_->projections_.size() + 1);
    A->reserve(expected_num_nonzero);
    for(int i = 0; i < dataset->descriptors_.cols(); ++i) {
      A->startVec(i);
      if(dataset->labels_(i) == -2)
	continue;

      double label = -1;
      if(dataset->labels_(i) == class_id)
	label = 1;
      
      A->insertBack(0, i) = -label; // For the prior.

      int start_idx = 1;
      for(int j = 0; j < dataset->descriptors_.rows(); ++j) {
	int idx = slicer_->projections_[j]->getCellIdx(dataset->descriptors_(j, i));
	if(idx != Projection::NO_CELL)
	  A->insertBack(start_idx + idx, i) = -label;
	start_idx += slicer_->projections_[j]->cells_.cols();
      }
    }
    A->finalize();
  }

  
  /************************************************************
   * LogisticStochasticTrainer
   ************************************************************/

  LogisticStochasticTrainer::LogisticStochasticTrainer() :
    Trainer(),
    scheduler_(NULL)
  {
  }

  LogisticStochasticTrainer::~LogisticStochasticTrainer()
  {
    if(scheduler_)
      delete scheduler_;
  }
  
  void LogisticStochasticTrainer::attachSlicer(ProjectionSlicer::Ptr slicer)
  {
    Trainer::attachSlicer(slicer);
    num_instances_seen_ = VectorXi::Zero(slicer_->prior_.rows());
  }
  
  void LogisticStochasticTrainer::train(Dataset::ConstPtr dataset)
  {
    ROS_ASSERT(slicer_);
    ROS_ASSERT(scheduler_);

    vector<int> index(dataset->labels_.rows());
    for(size_t i = 0; i < index.size(); ++i)
      index[i] = i;
    random_shuffle(index.begin(), index.end());
    
    for(size_t i = 0; i < index.size(); ++i) {
      int idx = index[i];
      if((int)i % (dataset->descriptors_.cols() / 10) == 0)
	ROS_DEBUG_STREAM("Completed training on " << i << " / " << dataset->descriptors_.cols() << " instances with stochastic logistic regression." << flush);

      if(dataset->labels_(idx) == -2)
	continue;

      scheduler_->incrementInstances();
      VectorXf desc = dataset->descriptors_.col(idx);
      VectorXf response = slicer_->classify(desc).response_;
      for(size_t j = 0; j < slicer_->getNumClasses(); ++j) {
	double y = -1.0;
	if(dataset->labels_(idx) == (int)j)
	  y = 1.0;

	step(desc, response(j), y, j);
      }
    }
  }

  void LogisticStochasticTrainer::step(const VectorXf& descriptor, double response, double y, int response_class_id)
  {

    double weight = 1.0 / (1.0 + exp(y * response));
    double update = scheduler_->getLearningRate() * y * weight;
    slicer_->prior_(response_class_id) += update;
    for(size_t i = 0; i < slicer_->projections_.size(); ++i) {
      Projection& proj = *slicer_->projections_[i];
      int idx = proj.getCellIdx(descriptor(i));
      if(idx == Projection::NO_CELL)
	continue;

      proj.cells_(response_class_id, idx) += update;
    }
  }
    

  /************************************************************
   * HybridTrainer
   ************************************************************/
  
  HybridTrainer::HybridTrainer() :
    NaiveTrainer()
  {
  }
  
  void HybridTrainer::trainProjectionWeights(Dataset::ConstPtr dataset)
  {
    ROS_ASSERT(slicer_);
    for(int i = 0; i < slicer_->offset_.rows(); ++i)
      ROS_ASSERT(fabs(slicer_->offset_(i)) < 1e-6);

    for(size_t i = 0; i < slicer_->getNumClasses(); ++i) {
      ROS_DEBUG_STREAM("Training projection weights for class " << i << flush);
      trainProjectionWeightsForClass(i, dataset);
    }
    ROS_DEBUG_STREAM("Done training projection weights." << flush);
  }

  void HybridTrainer::trainProjectionWeightsForClass(int class_id, Dataset::ConstPtr dataset)
  {
    // -- Set up data matrix.
    //    Much of this computation could be cached, but this is memory inefficient,
    //    and in practice has been the bottleneck.
    VectorXd labels = VectorXd::Ones(dataset->labels_.rows()) * -1;
    for(int i = 0; i < labels.rows(); ++i)
      if(dataset->labels_(i) == class_id)
	labels(i) = 1;

    HighResTimer hrt("Creating data matrix");
    hrt.start();
    shared_ptr<MatrixXd> A(new MatrixXd(dataset->descriptors_.rows() + 1, dataset->descriptors_.cols()));
    A->row(0) = -labels.transpose();
    VectorXf response_buf;
    for(int i = 0; i < dataset->descriptors_.cols(); ++i) {
      for(int j = 0; j < dataset->descriptors_.rows(); ++j) { 
	response_buf = slicer_->projections_[j]->classify(dataset->descriptors_(j, i));
	A->coeffRef(j+1, i) = response_buf(class_id) * A->coeffRef(0, i);
      }
    }
    hrt.stop();
    ROS_DEBUG_STREAM(hrt.reportSeconds() << flush);

    // -- Run logistic regression and install learned params.
    Logistic lg;
    lg.A_ = A;
    lg.max_num_iters_ = 100;
    lg.train();
    slicer_->projection_weights_.row(class_id) = lg.getWeights().transpose().cast<float>();
    slicer_->prior_(class_id) = lg.getIntercept();
  }

  
  /************************************************************
   * HybridStochasticTrainer
   ************************************************************/
  
  HybridStochasticTrainer::HybridStochasticTrainer() :
    NaiveTrainer(),
    scheduler_(NULL)
  {
  }

  HybridStochasticTrainer::~HybridStochasticTrainer()
  {
    if(scheduler_)
      delete scheduler_;
  }
  
  void HybridStochasticTrainer::trainProjectionWeights(Dataset::ConstPtr dataset)
  {
    ROS_ASSERT(slicer_);
    ROS_ASSERT(scheduler_);
    
    for(int i = 0; i < dataset->descriptors_.cols(); ++i) {
      if(dataset->labels_(i) == -2)
	continue;

      scheduler_->incrementInstances();
      VectorXf desc = dataset->descriptors_.col(i);
      VectorXf response = slicer_->classify(desc).response_;
      for(size_t j = 0; j < slicer_->getNumClasses(); ++j) {
	double y = -1.0;
	if((int)j == dataset->labels_(i))
	  y = 1.0;

	step(desc, response(j), y, j);
      }
    }
  }
  
  void HybridStochasticTrainer::step(const VectorXf& descriptor, double response, double y, int response_class_id)
  {
    double weight = 1.0 / (1.0 + exp(y * response));
    slicer_->offset_(response_class_id) += scheduler_->getLearningRate() * y * weight;
    
    for(size_t i = 0; i < slicer_->projections_.size(); ++i) {
      int idx = slicer_->projections_[i]->getCellIdx(descriptor(i));
      if(idx == Projection::NO_CELL)
	continue;
      
      VectorXf projection_response = slicer_->projections_[i]->classify(descriptor(i));
      slicer_->projection_weights_(response_class_id, i) += scheduler_->getLearningRate() * y * weight * projection_response(response_class_id);
    }
  }

} // namespace odontomachus
