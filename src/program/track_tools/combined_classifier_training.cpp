#include <unconstrained_optimization/unconstrained_optimization.h>
#include <track_manager.h>
#include <boost/filesystem.hpp>
#include <multibooster_support.h>
#include <track_descriptors.h>

using namespace std;
using boost::shared_ptr;
using namespace track_manager;
using namespace pipeline;
using namespace track_descriptors;
using namespace Eigen;

#define NUM_THREADS getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1

//! \sum_i a_i exp(b_i x), for scalar x.
class Objective : public ScalarFunction
{
public:
  VectorXd a_;
  VectorXd b_;
  Objective(const VectorXd& a, const VectorXd& b) :
    a_(a),
    b_(b)
  {
    assert(a_.rows() == b_.rows());
  }
  
  double eval(const VectorXd& x) const {
    assert(x.rows() == 1);
    double obj = 0.0;
    for(int i = 0; i < a_.rows(); ++i)
      obj += a_(i) * exp(b_(i) * x(0));
    obj /= (double)a_.rows();
    return obj;
  }
};
    
class Gradient : public VectorFunction
{
public:
  VectorXd a_;
  VectorXd b_;
  Objective(const VectorXd& a, const VectorXd& b) :
    a_(a),
    b_(b)
  {
    assert(a_.rows() == b_.rows());
  }

  double eval(const VectorXd& x) const {
    assert(x.rows() == 1);
    VectorXd grad = VectorXd::Zero(1);
    for(int i = 0; i < a_.rows(); ++i)
      grad(0) += a_(i) * b_(i) * exp(b_(i) * x(0));

    grad /= (double)a_.rows();
    return grad;
  }
};

class Hessian : public MatrixFunction
{
public:
  VectorXd a_;
  VectorXd b_;
  Objective(const VectorXd& a, const VectorXd& b) :
    a_(a),
    b_(b)
  {
    assert(a_.rows() == b_.rows());
  }

  double eval(const VectorXd& x) const {
    assert(x.rows() == 1);
    MatrixXd hess = MatrixXd::Zero(1, 1);
    for(int i = 0; i < a_.rows(); ++i)
      hess(0, 0) += a_(i) * b_(i) * b_(i) * exp(b_(i) * x(0));

    hess /= (double)a_.rows();
    return grad;
  }
};

void computeProblemData(const CombinedClassifierPipeline& ccp, const vector<string>& tms, VectorXf* a, VectorXf* b) {

  vector<double> as;
  vector<double> bs;
  
  for(size_t i = 0; i < tms.size(); ++i) {
    TrackManager tm(tms[i]);
    as.reserve(as.size() + tm.tracks_.size() * ccp.tcp_->getClassMap().size());
    bs.reserve(bs.size() + tm.tracks_.size() * ccp.tcp_->getClassMap().size());
    
    for(size_t j = 0; j < tm.tracks_.size(); ++j) {
      VectorXf track_response;
      vector<VectorXf> frame_responses;
      VectorXf response = ccp.classify(tm.tracks_[j], &track_response, &frame_responses);

      int label;
      if(tr.label_.compare("unlabeled") == 0)
	continue; 
      if(tr.label_.compare("background") == 0)
	label = -1;
      else
	label = ccp.tcp_->getClassMap().toId(tr.label_);

      for(size_t c = 0; c < ccp.tcp_->getClassMap().size(); ++c) {
	double y = -1;
	if((int)c == label)
	  y = 1;

	bs.push_back(-y * track_response(c));
	
	double mean_frame_response = 0.0;
	for(size_t k = 0; k < frame_responses.size(); ++k)
	  mean_frame_response += frame_responses[k](c) - ccp.frame_classifier_->prior_(c);
	mean_frame_response /= (double)frame_responses.size();
	as.push_back(exp(-y * mean_frame_response));
      }
    }
  }

  *a = VectorXf(as.size());
  *b = VectorXf(bs.size());
  assert(as.size() == bs.size());
  for(size_t i = 0; i < as.size(); ++i) { 
    a->coeffRef(i) = as[i];
    b->coeffRef(i) = bs[i];
  }
  
}

int main(int argc, char** argv) {

  if(argc < 4) {
    cout << "Usage: " << argv[0] << " FRAME_CLASSIFIER TRACK_CLASSIFIER TRACK_MANAGER [TRACK_MANAGER ...] WEIGHT" << endl;
    return 1;
  }

  string output_filename(argv[argc-1]);
  cout << "Frame classifier: " << argv[1] << endl;
  cout << "Track classifier: " << argv[2] << endl;
  cout << "Output file: " << output_filename << endl;
  vector<string> tms;
  for(int i = 3; i < argc - 1; ++i)
    tms.push_back(argv[i]);
  
  if(boost::filesystem::exists(output_filename)) {
    cout << "Output file " << output_filename << " already exists!  Aborting." << endl;
    return 2;
  }

  CombinedClassifierPipeline ccp(argv[1], argv[2], 1.0, NUM_THREADS);
  VectorXf a;
  VectorXf b;
  computeProblemData(ccp, tms, &a, &b);

  Objective obj(a, b);
  Gradient grad(a, b);
  Hessian hess(a, b);
  double tol = 1e-6;
  double alpha = 0.3;
  double beta = 0.5;
  double stepsize = 1.0;
  VectorXd init = VectorXd::Zero(1);
  
  NewtonSolver solver(&obj, &grad, &hess, tol, alpha, beta, stepsize, true);
  VectorXd alpha = solver.solve(init);
  
  serializeMatrix(alpha, output_filename);
  return 0;
}
  
