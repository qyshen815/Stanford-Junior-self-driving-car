#ifndef EDGE_POTENTIAL_GENERATOR_H
#define EDGE_POTENTIAL_GENERATOR_H

#include <Eigen/Eigen>
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Sparse>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pipeline2/pipeline2.h>
#include <dst/typedefs.h>

namespace dst
{

  inline
  int getIdx(int row, int col, int num_cols)
  {
    return col + row * num_cols;
  }
  
  //! Abstract base class for classes that compute edge potentials.
  class EdgePotentialGenerator : public pipeline2::ComputeNode
  {
  public:
    pipeline2::Outlet< Eigen::SparseMatrix<double, Eigen::RowMajor>* > edge_otl_;
    EdgePotentialGenerator();
    
  protected:
    //! potentials_(i, j) contains the potential from node id i to node id j.
    //! All node ids are in row major.
    Eigen::SparseMatrix<double, Eigen::RowMajor> potentials_;

    virtual std::string _getName() const = 0;
    void displayEdges(cv::Mat3b img) const;
  };

}

#endif // EDGE_POTENTIAL_GENERATOR_H
