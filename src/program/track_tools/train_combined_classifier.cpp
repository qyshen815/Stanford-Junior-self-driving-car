#include <unconstrained_optimization/unconstrained_optimization.h>
#include <unconstrained_optimization/common_functions.h>
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

void computeProblemData(CombinedClassifierPipeline& ccp, const vector<string>& tms, MatrixXd* a, VectorXd* b) {
//   if(boost::filesystem::exists("as.mat") && boost::filesystem::exists("bs.mat")) { 
//     cout << "Using cached problem data." << endl;
//     deserializeMatrix("as.mat", a);
//     deserializeVector("bs.mat", b);
//     cout << a->cols() << " terms." << endl;
//     return;
//   }
  
  vector<VectorXd> as;
  vector<double> bs;
  
  for(size_t i = 0; i < tms.size(); ++i) {
    cout << "Working on " << tms[i] << endl;
    TrackManager tm(tms[i]);
    as.reserve(as.size() + tm.tracks_.size() * ccp.tcp_->getClassMap().size());
    bs.reserve(bs.size() + tm.tracks_.size() * ccp.tcp_->getClassMap().size());
    
    for(size_t j = 0; j < tm.tracks_.size(); ++j) {
      if((int)j % ((int)tm.tracks_.size() / 10) == 0) { 
	cout << ".";
	cout.flush();
	usleep(1000);
      }
      
      VectorXf track_response;
      vector<VectorXf> frame_responses;
      ccp.classify(tm.tracks_[j], &track_response, &frame_responses);

      int label;
      Track& tr = *tm.tracks_[j];
      if(tr.label_.compare("unlabeled") == 0)
	continue; 
      if(tr.label_.compare("background") == 0)
	label = -1;
      else
	label = ccp.tcp_->getClassMap().toId(tr.label_);

      // -- Compute mean frame response.
      VectorXf mean_frame_response = VectorXf::Zero(ccp.frame_classifier_->prior_.rows());
      for(size_t k = 0; k < frame_responses.size(); ++k)
	mean_frame_response += 2.0 * (frame_responses[k] - ccp.frame_classifier_->prior_);
      mean_frame_response /= (double)frame_responses.size();

      // -- Add a and b for each class.
      for(size_t c = 0; c < ccp.tcp_->getClassMap().size(); ++c) {
	double y = -1;
	if((int)c == label)
	  y = 1;

	bs.push_back(0.0);

	VectorXd av(3);
	av(0) = -y * 2.0 * ccp.track_classifier_->prior_(c);
	av(1) = -y * 2.0 * (track_response(c) - ccp.track_classifier_->prior_(c));
	av(2) = -y * mean_frame_response(c);
	as.push_back(av);
      }
    }
  }

  *a = MatrixXd(as[0].rows(), as.size());
  *b = VectorXd(bs.size());
  assert(as.size() == bs.size());
  for(size_t i = 0; i < as.size(); ++i) { 
    a->col(i) = as[i];
    b->coeffRef(i) = bs[i];
  }
  
}

int main(int argc, char** argv) {
  if(argc < 4 || strstr(argv[argc-1], ".tm")) {
    cout << "Usage: " << argv[0] << " FRAME_CLASSIFIER TRACK_CLASSIFIER TRACK_MANAGER [TRACK_MANAGER ...] WEIGHT" << endl;
    return 1;
  }

  string frame_classifier_filename(argv[1]);
  string track_classifier_filename(argv[2]);
  string output_filename(argv[argc-1]);

  cout << "Frame classifier: " << frame_classifier_filename << endl;
  cout << "Track classifier: " << track_classifier_filename << endl;
  cout << "Output file: " << output_filename << endl;

  vector<string> tms;
  for(int i = 3; i < argc - 1; ++i)
    tms.push_back(argv[i]);
  
  if(boost::filesystem::exists(output_filename)) {
    cout << "Output file " << output_filename << " already exists!  Aborting." << endl;
    return 2;
  }

  VectorXd weights = VectorXd::Ones(3);
  CombinedClassifierPipeline ccp(frame_classifier_filename, track_classifier_filename, weights, NUM_THREADS);
  MatrixXd a;
  VectorXd b;
  cout << "Computing problem data." << endl;
  computeProblemData(ccp, tms, &a, &b);
//   serializeMatrix(a, "as.mat");
//   serializeVector(b, "bs.mat");
    
  ObjectiveMEL obj_mel(a / 2.0, b / 2.0); // MEL objective function solves for 1/2 log odds.  The 1/2 factor here makes it solve for log odds.
  GradientMEL grad_mel(a / 2.0, b / 2.0);
  HessianMEL hess_mel(a / 2.0, b / 2.0);
  ObjectiveMLS obj_mls(a, b); // MLL objective function solves for log odds.
  GradientMLS grad_mls(a, b);
  HessianMLS hess_mls(a, b);
  double tol = 1e-12;
  double alpha = 0.3;
  double beta = 0.5;
  double stepsize = 1.0;
  VectorXd init = VectorXd::Ones(3);

  cout << "Mean exponential loss:" << endl;
  NewtonSolver solver_mel(&obj_mel, &grad_mel, &hess_mel, tol, alpha, beta, stepsize, true);
  VectorXd weight_mel = solver_mel.solve(init);
  cout << "Newton learned weights of " << weight_mel.transpose() << endl;

//   NesterovGradientSolver solver_mel_nesterov(&obj_mel, &grad_mel, tol, alpha, beta);
//   VectorXd weight_mel_nesterov = solver_mel_nesterov.solve(init);
//   cout << "Nesterov learned weights of " << weight_mel_nesterov.transpose() << endl;
//   cout << "L2 distance: " << (weight_mel_nesterov - weight_mel).norm() << endl;
  
  cout << endl << "Mean logistic score:" << endl;

  NewtonSolver solver_mls(&obj_mls, &grad_mls, &hess_mls, tol, alpha, beta, stepsize, true);
  VectorXd weight_mls = solver_mls.solve(init);
  cout << "Newton learned weights of " << weight_mls << endl;

//   NesterovGradientSolver solver_mls_nesterov(&obj_mls, &grad_mls, tol, alpha, beta);
//   VectorXd weight_mls_nesterov = solver_mls_nesterov.solve(init);
//   cout << "Nesterov learned weights of " << weight_mls_nesterov.transpose() << endl;
//   cout << "L2 distance: " << (weight_mls_nesterov - weight_mls).norm() << endl;
  
  serializeVector(weight_mel, output_filename + "_mel");
  serializeVectorASCII(weight_mel, output_filename + "_mel.txt");

  serializeVector(weight_mls, output_filename);
  serializeVectorASCII(weight_mls, output_filename + ".txt");

  // -- Also solve for the equivalent problem with no knowledge of the holistic classifier.
  MatrixXd a2(a.rows() - 1, a.cols());
  int idx = 0;
  for(int i = 0; i < a.rows(); ++i) {
    if(i != 1) {
      a2.row(idx) = a.row(i);
      ++idx;
    }
  }
  
  VectorXd init2 = VectorXd::Ones(a2.rows());
  ObjectiveMLS obj_mls2(a2, b);
  GradientMLS grad_mls2(a2, b);
  HessianMLS hess_mls2(a2, b);
  NewtonSolver solver_mls2(&obj_mls2, &grad_mls2, &hess_mls2, tol, alpha, beta, stepsize, true);
  VectorXd weight_mls2 = solver_mls2.solve(init2);
  cout << "Ignoring holistic classifier, Newton learned weights of " << weight_mls2.transpose() << endl;
  
//   NesterovGradientSolver solver_mls_nesterov2(&obj_mls2, &grad_mls2, tol, alpha, beta, 0, 1, true);
//   VectorXd weight_mls_nesterov2 = solver_mls_nesterov2.solve(init);


  serializeVector(weight_mls2, output_filename + "_no_holistic");
  serializeVectorASCII(weight_mls2, output_filename + "_no_holistic.txt");
    
  return 0;
}
  
