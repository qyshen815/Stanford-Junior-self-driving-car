#ifndef SEED_DISTANCE_NPG_H
#define SEED_DISTANCE_NPG_H

#include <functional>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dst/depth_projector.h>
#include <dst/node_potential_generator.h>
#include <dst/typedefs.h>

namespace dst
{

  typedef std::pair<float, cv::Point2i> PP;

  class CustomPairCompare : public std::binary_function<PP, PP, bool>
  {
  public:
    bool operator()(const PP& lhs, const PP& rhs) const {return lhs.first > rhs.first;}
  };
  
  class SeedDistanceNPG : public NodePotentialGenerator
  {
  public:
    SeedDistanceNPG(pipeline2::Outlet<cv::Mat1b>* seed_otl,
		    pipeline2::Outlet<DepthProjector::Output>* index_otl,
		    float variance);

  protected:
    pipeline2::Outlet<cv::Mat1b>* seed_otl_;
    pipeline2::Outlet<DepthProjector::Output>* index_otl_;
    cv::Mat1f source_distances_;
    cv::Mat1f sink_distances_;
    float variance_;
    cv::Mat1b visited_;
    std::priority_queue<PP, std::vector<PP>, CustomPairCompare> pq_;

    void processNeighbor(const cv::Point2i& pt0,
			 const cv::Point2i& pt1,
			 cv::Mat1f distances,
			 KinectCloud::Ptr pcd,
			 cv::Mat1i index);
    cv::Mat1f visualizeDistanceMap(cv::Mat1f dist) const;
    void computeDistanceMap(int label, cv::Mat1f& distances);
    void _compute();
    void _display() const;
    void _flush();
    std::string _getName() const;
  };
  
}

#endif // SEED_DISTANCE_NPG_H
