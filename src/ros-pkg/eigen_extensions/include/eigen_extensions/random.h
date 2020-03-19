#ifndef EIGEN_EXTENSIONS_RANDOM_H
#define EIGEN_EXTENSIONS_RANDOM_H

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include <Eigen/Eigen>
#include <Eigen/Sparse>

namespace eigen_extensions
{
  void sampleSparseGaussianVector(int rows, int nnz, Eigen::SparseVector<double>* vec);
}


#endif // EIGEN_EXTENSIONS_RANDOM_H
