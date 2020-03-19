#include <multibooster_support.h>
#include <unconstrained_optimization/unconstrained_optimization.h>
#include "dbf_experiments.h"

using namespace std;
using namespace Eigen;

string usageString() {
  ostringstream oss;
  oss << "train_metaclassifier METACLASSIFIER_DATASET MULTIBOOSTER OUTPUT_PARAMS(binary) OUTPUT_PARAMS(ascii)" << endl;
  oss << " Assumes that there are 3 classes." << endl;
  return oss.str();
}

void computeA(const MatrixXf& dataset, const VectorXf& prior, MatrixXd* Aptr, VectorXd* bptr) {
  assert(Aptr);
  assert(bptr);

  // -- Initialize A and b.
  int track_id, frame_id, label;
  VectorXd frame_response, features, ymc;
  int num_classes = prior.rows();
  parseDataset(dataset, num_classes, 0, &track_id, &frame_id, &label, &frame_response, &features, &ymc);
  *Aptr = MatrixXd::Zero(num_classes * dataset.rows(), features.rows());
  *bptr = VectorXd::Zero(num_classes * dataset.rows());
  MatrixXd& A = *Aptr;
  VectorXd& b = *bptr;
  
  // -- Fill A and b.
  int idx = 0;
  VectorXd sum;
  double response0 = 0;
  for(int c = 0; c < prior.rows(); ++c) { 
    for(int i = 0; i < dataset.rows(); ++i, ++idx) {
      parseDataset(dataset, prior.rows(), i, &track_id, &frame_id, &label, &frame_response, &features, &ymc);

      // -- Track length normalization.
      //double normalizer = 1.0 / (1.0 + (double)frame_id);
      double normalizer = 1.0; // No track length normalization.
      
      // -- b
      if(frame_id == 0)
	response0 = frame_response(c);
      b(idx) = -ymc(c) * (prior(c) + normalizer * (response0 - prior(c)));

      // -- A
      if(frame_id == 0) {
	sum = VectorXd::Zero(features.rows());
	A.row(idx) = VectorXd::Zero(features.rows());
      }
      else {
	sum += -ymc(c) * (frame_response(c) - prior(c)) * features;
	A.row(idx) = normalizer * sum.transpose();
      }

    }
  }
  assert(idx == A.rows());
}

MatrixXf learnParams(MatrixXf* mat, const MultiBooster& mb) {

  // -- Set up Nesterov.
  double tol = 1e-4;
  if(getenv("TOL"))
    tol = atof(getenv("TOL"));

  double alpha = 0.15;
  if(getenv("ALPHA"))
    alpha = atof(getenv("ALPHA"));

  double beta = 0.2;
  if(getenv("BETA"))
    beta = atof(getenv("BETA"));

  cout << "Using alpha = " << alpha << ", beta = " << beta << ", tol = " << tol << endl;

  MatrixXd A;
  VectorXd b;
  computeA(*mat, 2.0*mb.prior_, &A, &b);
  ObjLogSig obj(A, b);
  GradLogSig grad(A, b);
  HessLogSig hess(A, b);
  int max_iters = 100;
  NewtonSolver newton(&obj, &grad, &hess, tol, alpha, beta, 1.0, 1, max_iters);

  // -- Solve.
  int num_features = mat->cols() - 6; //track, frame, label, car, ped, bike, features...
  VectorXd params = VectorXd::Zero(num_features);
  params(0) = 1; //Affine offset; start at the naive (but normalized) value.
  params = newton.solve(params);

  MatrixXf out = MatrixXf::Zero(params.rows(), 1);
  for(int i = 0; i < params.rows(); ++i)
    out(i, 0) = params(i);

  return out;
}

int main(int argc, char** argv) {
  if(argc == 5) {
    cout << "Loading metaclassifier dataset " << argv[1] << " using prior from " << argv[2] << ", saving metaclassifier params to " << argv[3] << endl;
    MultiBooster mb(argv[2]);

    MatrixXf training_set;
    deserializeMatrix(argv[1], &training_set);
    
    MatrixXf params = learnParams(&training_set, mb);
    cout << "Learned params: " << params.transpose() << endl;
    serializeMatrix(params, argv[3]);
    serializeMatrixASCII(params, argv[4]);
	
  }
  else {
    cout << usageString() << endl;
  }

  return 0;
}
