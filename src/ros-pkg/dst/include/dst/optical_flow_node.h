#ifndef OPTICAL_FLOW_NODE_H
#define OPTICAL_FLOW_NODE_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <pipeline2/pipeline2.h>
#include <dst/typedefs.h>
#include <dst/depth_projector.h>

namespace dst
{

  class OpticalFlowNode : public pipeline2::ComputeNode
  {
  public:
    typedef struct
    {
      cv::Mat3b img_;
      cv::Mat3b prev_img_;
      const std::vector<cv::Point2i>* prev_points_;
      const std::vector<cv::Point2i>* points_;
      //! status_[i] == 1 if a corresponding point has been found for prev_points_[i].
      //! 0 otherwise.
      const std::vector<uchar>* status_;
      //! How much this point looks like a depth edge, from 0 to 1, with 1 being the most like an edge.
      const std::vector<double>* edge_scores_;
    } Output;

    pipeline2::Outlet<Output> optflow_otl_;
    
    OpticalFlowNode(pipeline2::Outlet<DepthProjector::Output>* index_otl);
    static void safePointRound(const cv::Size& sz,
			       const cv::Point2f& fpt,
			       cv::Point2i* ipt);
			       
  protected:
    pipeline2::Outlet<DepthProjector::Output>* index_otl_;
    int max_corners_;
    double quality_level_;
    double min_distance_;
    int edge_radius_;
    double sigma_;
    std::vector<cv::Point2i> prev_points_;
    std::vector<cv::Point2i> points_;
    std::vector<cv::Point2f> prev_points_f_;
    std::vector<cv::Point2f> points_f_;
    //! edge_[i] is close to one if points_[i] is on a depth discontinuity, close to zero otherwise.
    //! exp(-max_gap / sigma_).
    std::vector<double> edge_scores_;
    std::vector<uchar> status_;
    std::vector<float> error_;
    Output output_;

    void _compute();
    void _display() const;
    void _flush();
    void _reset();
    std::string _getName() const;
  };
  
} // namespace dst

#endif // OPTICAL_FLOW_NODE_H
