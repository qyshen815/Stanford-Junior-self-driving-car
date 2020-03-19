#include <dst/segmentation_visualizer.h>

using namespace std;
using namespace pcl;
using namespace pcl::visualization;
using namespace dst;
namespace bfs = boost::filesystem;
using boost::shared_ptr;

namespace dst
{

  SegmentationVisualizer::SegmentationVisualizer(const std::vector<KinectCloud::Ptr>& original,
						 const std::vector<KinectCloud::Ptr>& segmented) :
    vis_("Segmentation"),
    original_(original),
    segmented_(segmented),
    idx_(0),
    stepping_(false)
  {
    ROS_ASSERT(!original_.empty());
    ROS_ASSERT(original_.size() == segmented_.size());
  
    vis_.registerKeyboardCallback(&SegmentationVisualizer::keyboardEventOccurred, *this);
    vis_.setBackgroundColor(255, 255, 255);
    PointCloudColorHandlerCustom<PointXYZRGB> single_color(original_[0], 0, 0, 0);
    vis_.addPointCloud(original_[0], single_color, "original");
    vis_.addPointCloud(segmented_[0], "segmented");
    vis_.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 1, "original");
    vis_.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 4, "segmented");
    
    // -- Hardcode the camera for the Kinect.
    vis_.camera_.clip[0] = 0.00387244;
    vis_.camera_.clip[1] = 3.87244;
    vis_.camera_.focal[0] = -0.160878;
    vis_.camera_.focal[1] = -0.0444743;
    vis_.camera_.focal[2] = 1.281;
    vis_.camera_.pos[0] = 0.0402195;
    vis_.camera_.pos[1] = 0.0111186;
    vis_.camera_.pos[2] = -1.7;
    vis_.camera_.view[0] = 0;
    vis_.camera_.view[1] = -1;
    vis_.camera_.view[2] = 0;
    vis_.camera_.window_size[0] = 1678;
    vis_.camera_.window_size[1] = 525;
    vis_.camera_.window_pos[0] = 2;
    vis_.camera_.window_pos[1] = 82;
    vis_.updateCamera();

    draw();
  }

  void SegmentationVisualizer::keyboardEventOccurred(const KeyboardEvent &event, void* data)
  {
    //PCLVisualizer& vis = *(PCLVisualizer*)data;
    if(event.getKeySym() == "w" && event.keyDown())
      increment(-1);
    else if(event.getKeySym() == "v" && event.keyDown())
      increment(1);
    else if(event.getKeySym() == "W" && event.keyDown())
      increment(-100);
    else if(event.getKeySym() == "V" && event.keyDown())
      increment(100);
    else if(event.getKeySym() == "s" && event.keyDown())
      stepping_ = !stepping_;

    draw();
  }

  void SegmentationVisualizer::draw()
  {
    ostringstream oss;
    oss << "Frame " << setw(4) << setfill('0') << idx_;
    vis_.removeShape("frame_number");
    vis_.addText(oss.str(), 10, 10, 0, 0, 0, "frame_number");
    vis_.updatePointCloud(segmented_[idx_], "segmented");
    PointCloudColorHandlerCustom<PointXYZRGB> single_color(original_[0], 0, 0, 0);
    vis_.updatePointCloud(original_[idx_], single_color, "original");
  }

  void SegmentationVisualizer::increment(int num)
  {
    idx_ += num;

    if(idx_ < 0)
      idx_ = 0;
    else if((size_t)idx_ >= segmented_.size()) { 
      idx_ = segmented_.size() - 1;
      stepping_ = false;
    }
  }

  void SegmentationVisualizer::spin()
  {
    while(!vis_.wasStopped()) { 
      vis_.spinOnce(15);
      usleep(15000);
      if(stepping_) { 
	increment(1);
	draw();
      }
    }
  }

} // namespace dst
