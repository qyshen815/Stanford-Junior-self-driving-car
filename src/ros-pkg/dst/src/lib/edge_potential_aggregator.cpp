#include <dst/edge_potentials.h>

using namespace std;
using namespace Eigen;

namespace dst
{
  
  EdgePotentialAggregator::EdgePotentialAggregator(pipeline2::Outlet<Graph3dPtr>* graph_otl,
						   pipeline2::Outlet<cv::Mat3b>* img_otl,
						   pipeline2::Outlet<DepthProjector::Output>* index_otl,
						   const std::vector<EdgePotentialGenerator*>& generators,
						   const Eigen::VectorXd& weights,
						   bool ignore_depthless,
						   pipeline2::Outlet< Eigen::SparseMatrix<double, Eigen::RowMajor>* >* weights_otl) :
    graph_otl_(graph_otl),
    img_otl_(img_otl),
    index_otl_(index_otl),
    generators_(generators),
    weights_(weights),
    ignore_depthless_(ignore_depthless),
    weights_otl_(weights_otl)
  {
    registerInput(graph_otl_->getNode());
    registerInput(img_otl_->getNode());
    registerInput(index_otl_->getNode());
    if(weights_otl_)
      registerInput(weights_otl_->getNode()); // TODO: What happens when you register an input twice?
    
    for(size_t i = 0; i < generators_.size(); ++i) { 
      registerInput(generators_[i]);
    }
  }

  void EdgePotentialAggregator::setWeights(const Eigen::VectorXd& weights)
  {
    ROS_ASSERT((size_t)weights.rows() == generators_.size());
    weights_ = weights;
  }

  void EdgePotentialAggregator::ignoreDepthless(const SparseMatrix<double, RowMajor>& pot,
						SparseMatrix<double, RowMajor>* sanitized) const
  {
    cv::Mat1i index = index_otl_->pull().current_index_;
    
    *sanitized = SparseMatrix<double, RowMajor>(pot.rows(), pot.cols());
    for(int i = 0; i < pot.outerSize(); ++i) { 
      SparseMatrix<double, RowMajor>::InnerIterator it(pot, i);
      sanitized->startVec(i);
      for(; it; ++it) {
	ROS_ASSERT(it.row() == i);
	
	int idx1 = it.row();
	int y1 = idx1 / index.cols;
	int x1 = idx1 - y1 * index.cols;
	int idx2 = it.col();
	int y2 = idx2 / index.cols;
	int x2 = idx2 - y2 * index.cols;
	if(index(y1, x1) == -1 || index(y2, x2) == -1)
	  continue;
	sanitized->insertBack(it.row(), it.col()) = it.value();
      }
    }
  }
    
  void EdgePotentialAggregator::cacheUnweightedPotentials(FramePotentialsCache::Ptr framecache) const
  {
    ROS_ASSERT(weights_otl_);
    ROS_ASSERT(framecache->edge_potentials_.empty());
    ROS_ASSERT(ignore_depthless_);
    
    SparseMatrix<double, RowMajor>& w = *weights_otl_->pull();
    for(size_t i = 0; i < generators_.size(); ++i) {
      ROS_ASSERT(generators_[i]->edge_otl_.pull());
      SparseMatrix<double, RowMajor> gen;
      ignoreDepthless(*generators_[i]->edge_otl_.pull(), &gen);
      if(generators_[i] == weights_otl_->getNode())
	framecache->edge_potentials_.push_back(gen);
      else
	framecache->edge_potentials_.push_back(gen.cwiseProduct(w));
    }
  }
  
  void EdgePotentialAggregator::_compute()
  {
    // -- Allocate more space if necessary.
    int num_nodes = graph_otl_->pull()->get_node_num();
    potentials_ = SparseMatrix<double, RowMajor>(num_nodes, num_nodes);
    potentials_.reserve(10 * num_nodes);

    // -- Weighted sum.
    if(weights_otl_) {
      SparseMatrix<double, RowMajor>& w = *weights_otl_->pull();
      for(size_t i = 0; i < generators_.size(); ++i) {
	SparseMatrix<double, RowMajor> gen;
	ignoreDepthless(*generators_[i]->edge_otl_.pull(), &gen);
	if(generators_[i] == weights_otl_->getNode())
	  potentials_ += weights_[i] * gen;
	else
	  potentials_ += weights_[i] * gen.cwiseProduct(w);
      }
    }
    else { 
      for(size_t i = 0; i < generators_.size(); ++i) {
	SparseMatrix<double, RowMajor> gen;
	ignoreDepthless(*generators_[i]->edge_otl_.pull(), &gen);
	potentials_ += weights_[i] * gen;
      }
    }
    
    edge_otl_.push(&potentials_);
  }

  void EdgePotentialAggregator::_display() const
  {
    displayEdges(img_otl_->pull());
  }

  void EdgePotentialAggregator::_flush()
  {
    potentials_.setZero();
    edge_otl_.push(NULL);
  }

  std::string EdgePotentialAggregator::_getName() const
  {
    return "EdgePotentialAggregator";
  }
  
} // namespace dst
