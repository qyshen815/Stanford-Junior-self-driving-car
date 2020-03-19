#ifndef EPNP_SOLVER_H
#define EPNP_SOLVER_H

#include <epnp/epnp.h>
#include <Eigen/Eigen>
#include <vector>

class EPnPSolver
{
public:
  EPnPSolver(double center_u, double center_v, double focal_length_u, double focal_length_v);
  void addCorrespondence(const Eigen::VectorXd& xyz, Eigen::VectorXd& uv);
  //! Returns a matrix A for which A * velo coords pt = camera coords pt.
  Eigen::MatrixXd solve();
  void clear();
  size_t numCorrespondences() const;
  
private:
  epnp epnp_;
  std::vector<Eigen::VectorXd> xyz_points_;
  std::vector<Eigen::VectorXd> uv_points_;
};

#endif // EPNP_SOLVER_H
