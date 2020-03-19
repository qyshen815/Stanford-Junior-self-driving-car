#ifndef NIPS_H
#define NIPS_H

#include <Eigen/Eigen>
#include <unconstrained_optimization/unconstrained_optimization.h>

double sigmoid(double z);
double logsig(double z);



void parseDatasetFloat(const Eigen::MatrixXf& mat,
		       int num_classes,
		       int idx,
		       int* track_id,
		       int* frame_id,
		       int* label,
		       Eigen::VectorXf* frame_response,
		       Eigen::VectorXf* features,
		       Eigen::VectorXf* ymc);


void parseDataset(const Eigen::MatrixXf& mat,
		      int num_classes,
		      int idx,
		      int* track_id,
		      int* frame_id,
		      int* label,
		      Eigen::VectorXd* frame_response,
		      Eigen::VectorXd* features,
		      Eigen::VectorXd* ymc);


class ObjLogSig : public ScalarFunction
{
public:
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
  
  ObjLogSig(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) :
    A_(A),
    b_(b)
  {
    assert(A_.rows() == b_.rows());
  }

  double eval(const Eigen::VectorXd& x) const {
    double obj = 0;
    for(int i = 0; i < A_.rows(); ++i)
      obj += -logsig(-A_.row(i).dot(x) - b_(i))  / (double)A_.rows();
    return obj;
  }
};

class GradLogSig : public VectorFunction
{
public:
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;

  GradLogSig(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) :
    A_(A),
    b_(b)
  {
    assert(A_.rows() == b_.rows());
  }
  
  Eigen::VectorXd eval(const Eigen::VectorXd& x) const {
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(x.rows());
    for(int i = 0; i < A_.rows(); ++i) {
      grad += sigmoid(x.dot(A_.row(i)) + b_(i)) * (A_.row(i).transpose())  / (double)A_.rows();
    }
    return grad;
  }
};

class HessLogSig : public MatrixFunction
{
public:
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;

  HessLogSig(const Eigen::MatrixXd& A, const Eigen::VectorXd& b) :
    A_(A),
    b_(b)
  {
    assert(A_.rows() == b_.rows());
  }
  
  Eigen::MatrixXd eval(const Eigen::VectorXd& x) const {
    Eigen::MatrixXd hess = Eigen::MatrixXd::Zero(x.rows(), x.rows());
    for(int i = 0; i < A_.rows(); ++i) {
      hess += sigmoid(-x.dot(A_.row(i)) - b_(i)) * sigmoid(x.dot(A_.row(i)) + b_(i)) * A_.row(i).transpose() * A_.row(i) / (double)A_.rows();
    }
    return hess;
  }
};


#endif // NIPS_H
