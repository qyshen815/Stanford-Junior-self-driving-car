#ifndef NODE_POTENTIAL_AGGREGATOR_H
#define NODE_POTENTIAL_AGGREGATOR_H

#include <opencv2/core/core.hpp>
#include <dst/node_potential_generator.h>
#include <dst/typedefs.h>
#include <dst/edge_potential_aggregator.h>

namespace dst
{
  
  //! Collects all node potentials, applies weights to them, and adds to
  //! the graph object.  Allocates the graph object.
  class NodePotentialAggregator : public NodePotentialGenerator
  {
  public:
    NodePotentialAggregator(pipeline2::Outlet<Graph3dPtr>* graph_otl,
			    pipeline2::Outlet<cv::Mat1b>* seed_otl,
			    pipeline2::Outlet<cv::Mat3b>* image_otl,
			    pipeline2::Outlet<DepthProjector::Output>* index_otl,
			    EdgePotentialAggregator* edge_aggregator,
			    const std::vector<NodePotentialGenerator*>& generators,
			    const Eigen::VectorXd& weights);
    ~NodePotentialAggregator();

    void setWeights(const Eigen::VectorXd& weights);
    Eigen::VectorXd getWeights() const { return weights_; }
    void cacheUnweightedPotentials(FramePotentialsCache::Ptr framecache) const;
    
  protected:
    pipeline2::Outlet<Graph3dPtr>* graph_otl_;
    pipeline2::Outlet<cv::Mat1b>* seed_otl_;
    //! For visualization only.
    pipeline2::Outlet<cv::Mat3b>* image_otl_;
    pipeline2::Outlet<DepthProjector::Output>* index_otl_;
    EdgePotentialAggregator* edge_aggregator_;
    std::vector<NodePotentialGenerator*> generators_;
    Eigen::VectorXd weights_;

    void ignoreDepthless(const Eigen::MatrixXd& pot,
			 Eigen::MatrixXd* sanitized) const;
    void _compute();
    //! Optional.
    void _display() const;
    void _flush();
    //! By convention, this is <classname>_<param><val>_<param><val>_...
    std::string _getName() const;
  };

}

#endif // NODE_POTENTIAL_AGGREGATOR_H
