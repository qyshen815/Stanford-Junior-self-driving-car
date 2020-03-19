#include <dst/node_potential_aggregator.h>

using namespace std;
using namespace Eigen;
using namespace pipeline2;

namespace dst
{
  
  NodePotentialAggregator::NodePotentialAggregator(Outlet<Graph3dPtr>* graph_otl,
						   Outlet<cv::Mat1b>* seed_otl,
						   Outlet<cv::Mat3b>* image_otl,
						   Outlet<DepthProjector::Output>* index_otl,
						   EdgePotentialAggregator* edge_aggregator,
						   const std::vector<NodePotentialGenerator*>& generators,
						   const Eigen::VectorXd& weights) :
    NodePotentialGenerator(),
    graph_otl_(graph_otl),
    seed_otl_(seed_otl),
    image_otl_(image_otl),
    index_otl_(index_otl),
    edge_aggregator_(edge_aggregator),
    generators_(generators),
    weights_(weights)
  {
    registerInput(graph_otl_->getNode());
    registerInput(seed_otl_->getNode());
    registerInput(image_otl_->getNode());
    registerInput(index_otl_->getNode());
    for(size_t i = 0; i < generators_.size(); ++i)
      registerInput(generators_[i]);
  }

  NodePotentialAggregator::~NodePotentialAggregator()
  {
  }

  
  void NodePotentialAggregator::setWeights(const Eigen::VectorXd& weights)
  {
    ROS_ASSERT((size_t)weights.rows() == generators_.size());
    weights_ = weights;
  }

  void NodePotentialAggregator::ignoreDepthless(const Eigen::MatrixXd& pot,
						Eigen::MatrixXd* sanitized) const
  {
    cv::Mat1i index = index_otl_->pull().current_index_;
    *sanitized = pot;
    for(int y = 0; y < sanitized->rows(); ++y)
      for(int x = 0; x < sanitized->cols(); ++x)
	if(index(y, x) == -1)
	  sanitized->coeffRef(y, x) = 0;
  }
  
  void NodePotentialAggregator::cacheUnweightedPotentials(FramePotentialsCache::Ptr framecache) const
  {
    ROS_ASSERT(framecache->sink_potentials_.empty());
    ROS_ASSERT(framecache->source_potentials_.empty());

    // -- Determine size.
    int rows = -1;
    int cols = -1;
    for(size_t i = 0; i < generators_.size(); ++i) {
      if(!generators_[i]->source_otl_.pull())
	continue;

      rows = generators_[i]->source_otl_.pull()->rows();
      cols = generators_[i]->source_otl_.pull()->cols();
      break;
    }
    ROS_ASSERT(rows != -1 && cols != -1);

    // -- Cache all potentials, setting to zero if none.
    for(size_t i = 0; i < generators_.size(); ++i) {
      if(!generators_[i]->source_otl_.pull()) {
	ROS_ASSERT(!generators_[i]->sink_otl_.pull());
	framecache->source_potentials_.push_back(MatrixXd::Zero(rows, cols));
	framecache->sink_potentials_.push_back(MatrixXd::Zero(rows, cols));
      }
      else {
	ROS_ASSERT(generators_[i]->sink_otl_.pull());
	
	MatrixXd sanitized_source;
	ignoreDepthless(*generators_[i]->source_otl_.pull(), &sanitized_source);
	framecache->source_potentials_.push_back(sanitized_source);

	MatrixXd sanitized_sink;
	ignoreDepthless(*generators_[i]->sink_otl_.pull(), &sanitized_sink);
	framecache->sink_potentials_.push_back(sanitized_sink);
      }
    }

    ROS_ASSERT(framecache->sink_potentials_.size() == framecache->source_potentials_.size());
  }

  void NodePotentialAggregator::_compute()
  {
    ROS_ASSERT((size_t)weights_.rows() == generators_.size());

    // -- Reinitialize the potential storage if needbe.
    int rows = -1;
    int cols = -1;
    for(size_t i = 0; i < generators_.size(); ++i) {
      if(!generators_[i]->source_otl_.pull())
	continue;

      rows = generators_[i]->source_otl_.pull()->rows();
      cols = generators_[i]->source_otl_.pull()->cols();
      break;
    }
    ROS_ASSERT(rows != -1 && cols != -1);
    
    if(source_potentials_.rows() != rows || source_potentials_.cols() != cols)
      source_potentials_ = MatrixXd::Zero(rows, cols);
    if(sink_potentials_.rows() != rows || sink_potentials_.cols() != cols)
      sink_potentials_ = MatrixXd::Zero(rows, cols);

    // -- Get weighted sum of source and sink potentials.
    for(size_t i = 0; i < generators_.size(); ++i) {
      if(!generators_[i]->source_otl_.pull())
	continue;

      MatrixXd sanitized_source;
      ignoreDepthless(*generators_[i]->source_otl_.pull(), &sanitized_source);
      source_potentials_ += weights_[i] * sanitized_source;      
      
      MatrixXd sanitized_sink;
      ignoreDepthless(*generators_[i]->sink_otl_.pull(), &sanitized_sink);
      sink_potentials_ += weights_[i] * sanitized_sink;      
    }
    
    // -- Load the outlets.
    source_otl_.push(&source_potentials_);
    sink_otl_.push(&sink_potentials_);
  }
  
  void NodePotentialAggregator::_display() const
  {
    displayNodePotentials(image_otl_->pull());
  }
  
  void NodePotentialAggregator::_flush()
  {
    source_potentials_.setZero();
    sink_potentials_.setZero();

    source_otl_.push(NULL);
    sink_otl_.push(NULL);
  }
  
  std::string NodePotentialAggregator::_getName() const
  {
    ostringstream oss;
    oss << "NodePotentialAggregator";
    return oss.str();
  }
  

  
} // namespace dst
