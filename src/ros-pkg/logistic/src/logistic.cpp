#include <logistic/logistic.h>

using namespace std;
using namespace Eigen;
using boost::shared_ptr;

Logistic::Logistic() :
  tol_(1e-6),
  alpha_(0.3),
  beta_(0.5),
  max_num_iters_(0),
  initial_stepsize_(1),
  debug_(true)
{
}

void Logistic::train(const Eigen::MatrixXd& descriptors, const Eigen::VectorXi& labels)
{
  ROS_ASSERT(descriptors.cols() == labels.rows());
  for(int i = 0; i < labels.rows(); ++i)
    ROS_ASSERT(labels.coeff(i) == -1 || labels.coeff(i) == 1);

  A_ = shared_ptr<MatrixXd>(new MatrixXd(descriptors.rows() + 1, descriptors.cols()));
  A_->row(0) = -labels.cast<double>().transpose();
  for(int i = 0; i < descriptors.rows(); ++i)
    A_->row(i+1) = descriptors.row(i).cwiseProduct(A_->row(0));

  train();
}

void Logistic::train()
{
  ROS_ASSERT(A_);

  if(!b_ || b_->rows() != A_->cols()) { 
    b_ = shared_ptr<VectorXd>(new VectorXd());
    *b_ = VectorXd::Zero(A_->cols());
  }
  
  ObjectiveMLSNoCopy objective(A_, b_);
  GradientMLSNoCopy gradient(A_, b_);

  if(getenv("USE_GRADIENT")) {
    cout << "Using gradient solver." << endl;
    
    GradientSolver gs(&objective, &gradient, tol_, alpha_, beta_, max_num_iters_, initial_stepsize_, debug_);
    HighResTimer hrt("Gradient");
    hrt.start();
    VectorXd output = gs.solve(VectorXd::Zero(A_->rows()));
    hrt.stop();
    cout << hrt.reportSeconds() << endl;
    
    intercept_ = output(0);
    weights_ = output.tail(output.rows() - 1);
  }
  else {
    NesterovGradientSolver ngs(&objective, &gradient, tol_, alpha_, beta_, max_num_iters_, initial_stepsize_, debug_);
    cout << "Using Nesterov solver." << endl;
    HighResTimer hrt("Nesterov");
    hrt.start();
    VectorXd output = ngs.solve(VectorXd::Zero(A_->rows()));
    hrt.stop();
    cout << hrt.reportSeconds() << endl;
    
    intercept_ = output(0);
    weights_ = output.tail(output.rows() - 1);
  }
}

Eigen::VectorXd Logistic::getWeights() const
{
  return weights_;
}

double Logistic::getIntercept() const
{
  return intercept_;
}

double Logistic::classify(const Eigen::VectorXd& descriptors) const
{
  return intercept_ + weights_.dot(descriptors);
}

void Logistic::serialize(std::ostream& out) const
{
  eigen_extensions::serialize(weights_, out);
  out.write((char*)&intercept_, sizeof(double));
}

void Logistic::deserialize(std::istream& in)
{
  eigen_extensions::deserialize(in, &weights_);
  in.read((char*)&intercept_, sizeof(double));
}

