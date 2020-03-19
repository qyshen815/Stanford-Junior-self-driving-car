#include "epnp_solver.h"

using namespace std;
using namespace Eigen;

EPnPSolver::EPnPSolver(double center_u, double center_v, double focal_length_u, double focal_length_v)
{
  epnp_.set_internal_parameters(center_u, center_v, focal_length_u, focal_length_v);
}

void EPnPSolver::addCorrespondence(const Eigen::VectorXd& xyz, Eigen::VectorXd& uv)
{
  xyz_points_.push_back(xyz);
  uv_points_.push_back(uv);
}

Eigen::MatrixXd EPnPSolver::solve()
{
  assert(xyz_points_.size() == uv_points_.size());

  epnp_.set_maximum_number_of_correspondences(xyz_points_.size());
  for(size_t i = 0; i < xyz_points_.size(); ++i) { 
    epnp_.add_correspondence(xyz_points_[i](0), xyz_points_[i](1), xyz_points_[i](2),
			     uv_points_[i](0), uv_points_[i](1));
  }
  
  
  double R[3][3];
  double T[3];
  epnp_.compute_pose(R, T);
  epnp_.print_pose(R, T);

  MatrixXd transform = MatrixXd::Identity(4,4);
  for(int i=0; i<3; ++i)
    for(int j=0; j<3; ++j)
      transform(i, j) = R[i][j];
  for(int i=0; i<3; ++i)
    transform(i, 3) = T[i];

  return transform;
}

void EPnPSolver::clear()
{
  xyz_points_.clear();
  uv_points_.clear();
}

size_t EPnPSolver::numCorrespondences() const
{
  return xyz_points_.size();
}
