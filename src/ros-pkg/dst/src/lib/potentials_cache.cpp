#include <dst/potentials_cache.h>

using namespace Eigen;

namespace dst
{

  int FramePotentialsCache::getNumWeights() const
  {
    ROS_ASSERT(sink_potentials_.size() == source_potentials_.size());
    return edge_potentials_.size() + source_potentials_.size();
  }

  double FramePotentialsCache::computeScore(cv::Mat1b labels,
					    const Eigen::VectorXd edge_weights,
					    const Eigen::VectorXd node_weights,
					    Eigen::VectorXd* psi) const
  {
    ROS_ASSERT((size_t)edge_weights.rows() == edge_potentials_.size());
    ROS_ASSERT((size_t)node_weights.rows() == source_potentials_.size());
    ROS_ASSERT((size_t)node_weights.rows() == sink_potentials_.size());
    double score = 0;
    if(psi)
      *psi = VectorXd::Zero(edge_weights.rows() + node_weights.rows());

    // -- Add up scores for edge potentials.
    for(size_t i = 0; i < edge_potentials_.size(); ++i) {
      const SparseMatrix<double, Eigen::RowMajor>& epot = edge_potentials_[i];
      SparseMatrix<double, Eigen::RowMajor> sym = (epot + epot.transpose()) / 2.0;
      for(int j = 0; j < sym.outerSize(); ++j) {
	for(SparseMatrix<double, RowMajor>::InnerIterator it(sym, j); it; ++it) {
	  if(it.col() <= it.row())
	    continue;

	  int idx1 = it.row();
	  int y1 = idx1 / labels.cols;
	  int x1 = idx1 - y1 * labels.cols;
	  int idx2 = it.col();
	  int y2 = idx2 / labels.cols;
	  int x2 = idx2 - y2 * labels.cols;

	  // Ignore pixels that don't have depth.
	  if(labels(y1, x1) != 255 && labels(y1, x1) != 0)
	    continue;
	  if(labels(y1, x1) == labels(y2, x2)) { 
	    score += it.value() * edge_weights(i);
	    if(psi)
	      psi->coeffRef(i) += it.value();
	  }
	}
      }
    }

    // -- Add up scores for node potentials.
    int idx = edge_potentials_.size();
    for(size_t i = 0; i < source_potentials_.size(); ++i, ++idx) {
      for(int y = 0; y < labels.rows; ++y) {
	for(int x = 0; x < labels.cols; ++x) {
	  if(labels(y, x) ==  255) { 
	    score += node_weights(i) * source_potentials_[i](y, x);
	    if(psi)
	      psi->coeffRef(idx) += source_potentials_[i](y, x);
	  }
	  else if(labels(y, x) == 0) { 
	    score += node_weights(i) * sink_potentials_[i](y, x);
	    if(psi)
	      psi->coeffRef(idx) += sink_potentials_[i](y, x);
	  }
	}
      }
    }

    return score;
  }
  
} // namespace dst
