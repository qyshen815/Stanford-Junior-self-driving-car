#ifndef POTENTIALS_CACHE_H
#define POTENTIALS_CACHE_H

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Sparse>
#include <opencv2/core/core.hpp>
#include <boost/shared_ptr.hpp>
#include <ros/assert.h>

namespace dst
{

  class FramePotentialsCache
  {
  public:
    typedef boost::shared_ptr<FramePotentialsCache> Ptr;

    //! In order of weights.
    std::vector< Eigen::SparseMatrix<double, Eigen::RowMajor> > edge_potentials_;
    //! In order of weights.
    std::vector<Eigen::MatrixXd> sink_potentials_;
    std::vector<Eigen::MatrixXd> source_potentials_;
    cv::Mat1i depth_index_;

    int getNumWeights() const;
    int getNumEdgeWeights() const { return edge_potentials_.size(); }
    double computeScore(cv::Mat1b labels,
			const Eigen::VectorXd edge_weights,
			const Eigen::VectorXd node_weights,
			Eigen::VectorXd* psi = NULL) const;
	
  };

  class PotentialsCache
  {
  public:
    typedef boost::shared_ptr<PotentialsCache> Ptr;

    std::vector<FramePotentialsCache::Ptr> framecaches_;
    int size() const { return framecaches_.size(); }
  };

}

#endif // POTENTIALS_CACHE_H
