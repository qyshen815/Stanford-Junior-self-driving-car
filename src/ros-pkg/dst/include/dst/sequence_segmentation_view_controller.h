#ifndef SEQUENCE_SEGMENTATION_VIEW_CONTROLLER_H
#define SEQUENCE_SEGMENTATION_VIEW_CONTROLLER_H

#include <image_labeler/opencv_view.h>
#include <pcl/visualization/cloud_viewer.h>
#include <dst/kinect_sequence.h>
#include <dst/lockable.h>
#include <dst/segmentation_pipeline.h>

namespace dst
{  
  class SequenceSegmentationViewController : public OpenCVViewDelegate, public Lockable
  {
  public:
    typedef enum {RAW, SEED, SEGMENTED} state_t;
    
    KinectSequence::Ptr seq_;
    OpenCVView seg_view_;
    pcl::visualization::CloudViewer pcd_view_;
    OpenCVView img_view_;
    int seed_radius_;

    SequenceSegmentationViewController(KinectSequence::Ptr seq);
    ~SequenceSegmentationViewController();
    void run();
    void setWeights(const Eigen::VectorXd& weights) { sp_.setWeights(weights); }

  private:
    SegmentationPipeline sp_;
    int current_idx_;
    bool quitting_;
    bool needs_redraw_;
    state_t state_;
    cv::Mat3b seed_vis_;
    cv::Mat3b seg_vis_;
    KinectCloud::Ptr seg_pcd_;
    bool show_depth_;
    
    void useSegmentationAsSeed();
    void toggleDebug();
    void saveGraphviz() const;
    void handleKeypress(char key);
    void mouseEvent(int event, int x, int y, int flags, void* param);
    void advance(int num);
    void clearHelperSeedLabels();
    void draw();
    void drawSegVis();
    void drawSeedVis();
    void segmentImage();
    void segmentSequence();
    void saveSequence() const;
    size_t size() const;
    void transitionTo(state_t state);
    
    friend class OpenCVView;
  };
    
  
} // namespace dst

#endif // SEQUENCE_SEGMENTATION_VIEW_CONTROLLER_H


