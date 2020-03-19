#include <unconstrained_optimization/unconstrained_optimization.h>


using namespace std;
using namespace Eigen;

double sigmoid(double z) {
  long double big = exp(-z);
  if(isinf(1.0 + big))
    return 0.0;
  else
    return 1.0 / (1.0 + big);
}

double logsig(double z) {
  long double big = exp(-z);
  if(isinf(1.0 + big))
    return z;
  else
    return -log(1.0 + big);
}

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

int main(int argc, char** argv) {
  cout << "Testing optimization algorithms." << endl;
  
  double tol = 1e-6;
  if(getenv("TOL"))
    tol = atof(getenv("TOL"));

  double alpha = 0.3;
  if(getenv("ALPHA"))
    alpha = atof(getenv("ALPHA"));

  double beta = 0.5;
  if(getenv("BETA"))
    beta = atof(getenv("BETA"));

  int n = 3;
  if(getenv("N"))
    n = atoi(getenv("N"));

  int m = 1000;
  if(getenv("M"))
    m = atoi(getenv("M"));

  double stepsize = 1.0;
  if(getenv("STEPSIZE"))
    stepsize = atof(getenv("STEPSIZE"));
  
  MatrixXd A = MatrixXd::Random(m, n);
  //VectorXd b = VectorXd::Random(m) * 10.0;
  VectorXd b = VectorXd::Zero(m);
  ObjLogSig obj(A, b);
  GradLogSig grad(A, b);
  HessLogSig hess(A, b);
  //VectorXd init = VectorXd::Zero(n);
  VectorXd init = VectorXd::Random(n) * 100;

  bool debug = false;

  timeval start, end;
  gettimeofday(&start, NULL);
  NewtonSolver ns(&obj, &grad, &hess, tol, alpha, beta, stepsize, debug);
  VectorXd soln2 = ns.solve(init);
  gettimeofday(&end, NULL);
  cout << "Solution: " << soln2.transpose() << endl;
  cout << " Finished in " <<   1e3*(end.tv_sec - start.tv_sec) + 1e-3*(end.tv_usec - start.tv_usec) << " ms." << endl;

  int max_num_iters = 0;
  gettimeofday(&start, NULL);
  NesterovGradientSolver ngs(&obj, &grad, tol, alpha, beta, max_num_iters, stepsize, debug);
  VectorXd soln = ngs.solve(init);
  gettimeofday(&end, NULL);
  cout << "Solution: " << soln.transpose() << endl;
  cout << " Finished in " <<   1e3*(end.tv_sec - start.tv_sec) + 1e-3*(end.tv_usec - start.tv_usec) << " ms." << endl;
}
