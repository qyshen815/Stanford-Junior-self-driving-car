#ifndef SCENE_ALIGNMENT_NODE_H
#define SCENE_ALIGNMENT_NODE_H

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/organized_neighbor_search.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Eigen>
#include <dst/kdtree_node.h>
#include <dst/optical_flow_node.h>

namespace dst
{

  class SceneAlignmentNode : public pipeline2::ComputeNode
  {
  public:
    //pipeline2::Outlet<const Eigen::Affine3f*> transform_otl_;
    //! The current cloud, transformed to overlap with the previous cloud.
    pipeline2::Outlet<KinectCloud::Ptr> transformed_otl_;
    
    SceneAlignmentNode(pipeline2::Outlet<KdTreeNode::Output>* kdtree_otl,
		       pipeline2::Outlet<OpticalFlowNode::Output>* optflow_otl,
		       pipeline2::Outlet<DepthProjector::Output>* index_otl,
		       double distance_threshold = 0.02,
		       double edge_threshold = 0.1,
		       int num_samples = 200);
    
  protected:
    pipeline2::Outlet<KdTreeNode::Output>* kdtree_otl_;
    pipeline2::Outlet<OpticalFlowNode::Output>* optflow_otl_;
    pipeline2::Outlet<DepthProjector::Output>* index_otl_;
    double distance_threshold_;
    double edge_threshold_;
    int num_samples_;
    //! Only consider optical flow points that look less like an edge than this.  0 to 1.
    std::vector<bool> valid_;
    std::vector<pcl::PointXYZRGB> previous_points_;
    std::vector<pcl::PointXYZRGB> current_points_;
    std::vector<pcl::PointXYZRGB> prev_inliers_;
    std::vector<pcl::PointXYZRGB> curr_inliers_;
    Eigen::Affine3f transform_;
    int best_score_;

    void get3DPoints(const std::vector<cv::Point2i>& img_pts,
		     const std::vector<bool>& valid,
		     cv::Mat1i index,
		     KinectCloud::Ptr pcd,
		     std::vector<pcl::PointXYZRGB>* pts) const;
    double scoreTransform(const Eigen::Affine3f& trans,
			  std::vector<pcl::PointXYZRGB>* curr_inliers,
			  std::vector<pcl::PointXYZRGB>* prev_inliers) const;
    Eigen::Affine3f computeTransform(const std::vector<pcl::PointXYZRGB>& prev_points,
				     const std::vector<pcl::PointXYZRGB>& curr_points) const;
    void sampleCorrespondences(std::vector<pcl::PointXYZRGB>* prev,
			       std::vector<pcl::PointXYZRGB>* curr) const;
    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };
  
}

#endif // SCENE_ALIGNMENT_NODE_H
