#include <dst/organized_surface_normal_node.h>
#include <bag_of_tricks/high_res_timer.h>

using namespace std;
using namespace Eigen;

namespace dst
{

  OrganizedSurfaceNormalNode::OrganizedSurfaceNormalNode(pipeline2::Outlet<DepthProjector::Output>* index_otl,
							 int radius) :
    ComputeNode(),
    normals_otl_(this),
    index_otl_(index_otl),
    radius_(radius),
    normals_(new Normals())
  {
    registerInput(index_otl_->getNode());
  }

  void OrganizedSurfaceNormalNode::computeNormal(const KinectCloud& pcd,
						 const pcl::PointXYZRGB& center,
  						 const std::vector<int>& indices,
  						 pcl::Normal* normal)
  {
    weights_.clear();
    weights_.resize(indices.size(), 0);
    double total_weight = 0;
    valid_.clear();
    valid_.resize(indices.size(), true);

    Vector3f mean = Vector3f::Zero();
    float num_valid = 0;
    for(size_t i = 0; i < indices.size(); ++i) {
      const Vector3f& pt = pcd[indices[i]].getVector3fMap();
      if(isnan(pt(0)) || isnan(pt(1)) || isnan(pt(2))) {
	valid_[i] = false;
  	continue;
      }

      double dist = pcl::euclideanDistance(pcd[indices[i]], center);
      weights_[i] = exp(-dist / 0.01);
      //weights_[i] = 1.0;
      total_weight += weights_[i];
      mean += pt;
      ++num_valid;
    }
    mean /= num_valid;
    
    // -- Normalize the weights.  - This is only necessary if there are numerical issues
    // or if we want valid curvature estimates.
    for(size_t i = 0; i < weights_.size(); ++i)
      weights_[i] /= total_weight;
    
    Matrix3f X = Matrix3f::Zero();
    for(size_t i = 0; i < indices.size(); ++i) {
      if(!valid_[i])
	continue;
      Vector3f pt = weights_[i] * (pcd[indices[i]].getVector3fMap() - center.getVector3fMap());
      //Vector3f pt = weights_[i] * (pcd[indices[i]].getVector3fMap() - mean);
      X += pt * pt.transpose();
    }

    pcl::solvePlaneParameters(X, 
			      normal->normal[0],
			      normal->normal[1],
			      normal->normal[2],
			      normal->curvature);
    
    pcl::flipNormalTowardsViewpoint(center, 0, 0, 0,
				    normal->normal[0],
				    normal->normal[1],
				    normal->normal[2]);
  }
  
  void OrganizedSurfaceNormalNode::computeNormal(const KinectCloud& pcd,
						 const pcl::PointXYZRGB& pt,
						 const cv::Point2i& img_pt,
						 pcl::Normal* normal)
  {
    indices_.clear();
    inliers_.clear();
    
    ImageRegionIterator it(cv::Size(pcd.width, pcd.height),
			   img_pt, radius_);
    for(; !it.done(); ++it) {
      int idx = it.indexRowMajor();
      
      if(isnan(pcd[idx].z))
	continue;

      indices_.push_back(it.indexRowMajor());
    }

    computeNormal(pcd, pt, indices_, normal);
  }
  
  void OrganizedSurfaceNormalNode::_compute()
  {
    KinectCloud::Ptr pcd = index_otl_->pull().current_pcd_;
    ROS_ASSERT(normals_->empty());
    normals_->resize(pcd->size());
    
    for(size_t y = 0; y < pcd->height; ++y) {
      for(size_t x = 0; x < pcd->width; ++x) {
	int idx = y * pcd->width + x;
	cv::Point2i img_pt(x, y);
	computeNormal(*pcd, pcd->at(idx), img_pt, &normals_->at(idx));
      }
    }
      
    normals_otl_.push(normals_);
  }

  void OrganizedSurfaceNormalNode::normalToColor(const pcl::Normal& p,
						 cv::Vec3b* color) const
  {
    cv::Vec3b& c = *color;
    if(!isfinite(p.normal[0]) || !isfinite(p.normal[1]) || !isfinite(p.normal[2])) { 
      c = cv::Vec3b(0, 0, 0);
      return;
    }
    
    ROS_ASSERT(p.normal[0] <= 1.0 && p.normal[0] >= -1.0);
    c[0] = fabs(p.normal[0]) * 255;
    c[1] = fabs(p.normal[1]) * 255;
    c[2] = fabs(p.normal[2]) * 255;
  }
  
  void OrganizedSurfaceNormalNode::_display() const
  {
    cv::Mat1i index = index_otl_->pull().current_index_;
    cv::Mat3b vis(index.size(), cv::Vec3b(0, 0, 0));

    for(int y = 0; y < vis.rows; ++y) {
      for(int x = 0; x < vis.cols; ++x) {
	if(index(y, x) == -1)
	  continue;

	normalToColor(normals_->at(index(y, x)), &vis(y, x));
      }
    }

    cv::imwrite("debug/" + getRunName() + ".png", vis);

    KinectCloud::Ptr pcd = index_otl_->pull().current_pcd_;
    pcl::PointCloud<pcl::PointXYZRGBNormal> cn;
    pcl::concatenateFields(*pcd, *normals_, cn);
    pcl::io::savePCDFileBinary("debug/" + getRunName() + ".pcd", cn);
  }

  void OrganizedSurfaceNormalNode::_flush()
  {
    indices_.clear();
    inliers_.clear();
    normals_->clear();
    normals_otl_.push(Normals::Ptr());
  }

  std::string OrganizedSurfaceNormalNode::_getName() const
  {
    std::ostringstream oss;
    oss << "OrganizedSurfaceNormalNode";
    return oss.str();
  }
    
} // namespace dst
