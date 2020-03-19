#include <dst/seed_image_distance_npg.h>

using namespace std;
using namespace Eigen;

namespace dst
{

  SeedDistanceNPG::SeedDistanceNPG(pipeline2::Outlet<cv::Mat1b>* seed_otl,
				   pipeline2::Outlet<DepthProjector::Output>* index_otl,
				   float variance) :
    NodePotentialGenerator(),
    seed_otl_(seed_otl),
    index_otl_(index_otl),
    variance_(variance)
  {
    registerInput(seed_otl_->getNode());
    registerInput(index_otl_->getNode());
  }

  cv::Mat1f SeedDistanceNPG::visualizeDistanceMap(cv::Mat1f dist) const
  {
    cv::Mat1f vis(dist.size());

    for(int y = 0; y < vis.rows; ++y) 
      for(int x = 0; x < vis.cols; ++x)
	vis(y, x) = exp(-dist(y, x) / variance_);

    return vis;
  }

  inline
  void SeedDistanceNPG::processNeighbor(const cv::Point2i& pt0,
					const cv::Point2i& pt1,
					cv::Mat1f distances,
					KinectCloud::Ptr pcd,
					cv::Mat1i index)
  {
    // TODO: Valgrind complains occasionally about this being an invalid read.
    if(pt1.x < 0 || pt1.x > distances.cols - 1 || 
       pt1.y < 0 || pt1.y > distances.rows - 1)
      return;
    if(visited_(pt1) == 255)
      return;

    cv::Point2i nearest0 = DepthProjector::findNearest(index, 2, pt0);
    cv::Point2i nearest1 = DepthProjector::findNearest(index, 2, pt1);

    // -- Compute the distance to neighbors through this node.
    double d;
    if((nearest0.x == -1 && nearest0.y == -1) ||
       (nearest1.x == -1 && nearest1.y == -1)) {
      d = 1.0; // Unreasonably large distance if there is no depth information.
    }
    else {
      ROS_ASSERT(pcd->size() == pcd->points.size());
      ROS_ASSERT(index(nearest0) >= 0 && (size_t)index(nearest0) < pcd->size());
      ROS_ASSERT(index(nearest1) >= 0 && (size_t)index(nearest1) < pcd->size());
      d = pcl::euclideanDistance(pcd->at(index(nearest0)), pcd->at(index(nearest1)));
    }
    
    float alt = distances(pt0) + d;
    if(alt < distances(pt1)) { 
      distances(pt1) = alt;
      pq_.push(PP(alt, pt1));
    }
  }
  
  void SeedDistanceNPG::computeDistanceMap(int label, cv::Mat1f& distances)
  {
    ROS_ASSERT(pq_.empty());

    cv::Mat1b seed = seed_otl_->pull();
    if(visited_.rows != seed.rows)
      visited_ = cv::Mat1b(seed.size());
    if(distances.rows != seed.rows)
      distances = cv::Mat1f(seed.size());

    visited_ = 0;
    distances = numeric_limits<float>::max();
    
    for(int y = 0; y < seed.rows; ++y) { 
      for(int x = 0; x < seed.cols; ++x) {
	if(seed(y, x) == label) { 
	  distances(y, x) = 0;
	  pq_.push(PP(0, cv::Point2i(x, y)));
	  visited_(y, x) = 255;
	}
      }
    }

    KinectCloud::Ptr pcd = index_otl_->pull().current_pcd_;
    cv::Mat1i index = index_otl_->pull().current_index_;
    ROS_ASSERT(index.rows > 0);
    while(!pq_.empty()) {
      PP top = pq_.top();
      pq_.pop();
      float dist = top.first;
      cv::Point2i pt = top.second;
      if(dist == numeric_limits<float>::max())
	break;

      // Only process if this entry is new.
      ROS_ASSERT(dist >= distances(pt));
      if(dist == distances(pt)) {
	for(int dx = -1; dx <= 1; ++dx) { 
	  for(int dy = -1; dy <= 1; ++dy) {
	    if(dx == 0 && dy == 0)
	      continue;
	    processNeighbor(pt, cv::Point2i(pt.x + dx, pt.y + dy), distances, pcd, index);
	  }
	}
	visited_(pt) = 255;
      }
    }

    // If there is anything left in the queue, get rid of it.
    // Both src & sink computation use it.
    while(!pq_.empty())
      pq_.pop();
  }

  void SeedDistanceNPG::_compute()
  {
    cv::Mat1b seed = seed_otl_->pull();
    
    if(seed.rows != source_potentials_.rows() ||
       seed.cols != source_potentials_.cols()) {
      source_potentials_ = MatrixXd::Zero(seed.rows, seed.cols);
      sink_potentials_ = MatrixXd::Zero(seed.rows, seed.cols);
    }

    computeDistanceMap(255, source_distances_);
    computeDistanceMap(0, sink_distances_);
    
    // -- Assign potentials.
    for(int y = 0; y < seed.rows; ++y) {
      for(int x = 0; x < seed.cols; ++x) {
	// double delta = (source_distances_(y, x) - sink_distances_(y, x)) / variance_;
	// source_potentials_(y, x) = 1.0 / (1.0 + exp(delta));
	// sink_potentials_(y, x) = 1.0 - source_potentials_(y, x);
	source_potentials_(y, x) = exp(-source_distances_(y, x) / variance_);
	sink_potentials_(y, x) = exp(-sink_distances_(y, x) / variance_);
      }
    }

    source_otl_.push(&source_potentials_);
    sink_otl_.push(&sink_potentials_);
  }

  void SeedDistanceNPG::_display() const
  {
    displayNodePotentials();
  }

  void SeedDistanceNPG::_flush()
  {    
    source_potentials_.setZero();
    sink_potentials_.setZero();
    
    source_otl_.push(NULL);
    sink_otl_.push(NULL);
  }

  string SeedDistanceNPG::_getName() const
  {
    ostringstream oss;
    oss << "SeedDistanceNPG_variance:" << variance_;
    return oss.str();
  }

} // namespace dst
