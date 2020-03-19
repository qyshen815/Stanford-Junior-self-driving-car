#include "dbf_experiments.h"

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

void parseDatasetFloat(const MatrixXf& mat,
		       int num_classes,
		       int idx,
		       int* track_id,
		       int* frame_id,
		       int* label,
		       VectorXf* frame_response,
		       VectorXf* features,
		       VectorXf* ymc)
{
  *track_id = mat(idx, 0);
  *frame_id = mat(idx, 1);
  *label = mat(idx, 2);

  assert(num_classes == 3);
  *frame_response = 2.0 * mat.block(idx, 3, 1, num_classes).transpose(); //Factor of 1/2 is corrected for here.
  assert(frame_response->rows() == 3 && frame_response->cols() == 1);

  *features = mat.block(idx, 3 + num_classes, 1, mat.cols() - (3 + num_classes)).transpose();
  assert(features->cols() == 1);
  
  *ymc = -VectorXf::Ones(num_classes);
  assert(*label > -2 && *label < 3);
  if(*label > -1)
    ymc->coeffRef(*label) = 1;
}
  

void parseDataset(const MatrixXf& mat,
		      int num_classes,
		      int idx,
		      int* track_id,
		      int* frame_id,
		      int* label,
		      VectorXd* frame_response,
		      VectorXd* features,
		      VectorXd* ymc)
  
{
  VectorXf frame_response_f;
  VectorXf features_f;
  VectorXf ymc_f;
  parseDatasetFloat(mat, num_classes, idx, track_id, frame_id, label, &frame_response_f, &features_f, &ymc_f);
  *frame_response = frame_response_f.cast<double>();
  *features = features_f.cast<double>();
  *ymc = ymc_f.cast<double>();
}
