#ifndef BILATERAL_NPG_H
#define BILATERAL_NPG_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dst/node_potential_generator.h>
#include <dst/kdtree_node.h>
#include <dst/scene_alignment_node.h>
#include <dst/image_region_iterator.h>

namespace dst
{

  class BilateralNPG : public NodePotentialGenerator
  {
  public:
    BilateralNPG(pipeline2::Outlet<KdTreeNode::Output>* kdtree_otl,
		 pipeline2::Outlet<KinectCloud::Ptr>* transformed_otl,
		 pipeline2::Outlet<cv::Mat1b>* prev_seg_otl,
		 pipeline2::Outlet<DepthProjector::Output>* index_otl,
		 double sigma_dist,
		 double sigma_color,
		 double sigma_img_dist);

  protected:
    pipeline2::Outlet<KdTreeNode::Output>* kdtree_otl_;
    pipeline2::Outlet<KinectCloud::Ptr>* transformed_otl_;
    pipeline2::Outlet<cv::Mat1b>* prev_seg_otl_;
    pipeline2::Outlet<DepthProjector::Output>* index_otl_;
    double sigma_dist_;
    double sigma_color_;
    double sigma_img_dist_;
    double img_radius_;

    std::vector<int> indices_;
    std::vector<float> distances_;
    std::vector<cv::Point2i> neighborhood_;

    void getNeighborhood(cv::Mat3b img,
			 const cv::Point2i& pt,
			 double radius,
			 std::vector<cv::Point2i>* neighborhood) const;

    double computeTerm(const pcl::PointXYZRGB& curr_pt,
		       const pcl::PointXYZRGB& prev_pt) const;
    double computeTerm(const cv::Point2i& curr_pt,
		       const cv::Vec3b& curr_color,
		       const cv::Point2i& prev_pt,
		       const cv::Vec3b& prev_color) const;
    double computeBilateral(const pcl::PointXYZRGB& curr_pt,
			    const KdTreeNode::KdTreeType& prev_kdtree,
			    const std::vector<cv::Point2i>& prev_rindex,
			    cv::Mat1b prev_seg,
			    const KinectCloud& prev_pcd);
    double computeBilateralNoDepth(const cv::Point2i& curr_pt,
				   cv::Mat3b curr_img,
				   cv::Mat3b prev_img,
				   cv::Mat1b prev_seg);
      
    void _compute();
    void _display() const;
    void _flush();
    //! Clears the cached pointclouds and trees.
    void _reset();
    std::string _getName() const;
  };
  
}

#endif // BILATERAL_NPG_H
