#include <online_learning/projection_slicer.h>

using namespace std;
using namespace Eigen;
namespace fs = boost::filesystem;

namespace odontomachus
{

  Evaluator::Evaluator(ProjectionSlicer::Ptr slicer) :
    slicer_(slicer)
  {
  }
  
  void Evaluator::evaluate(Dataset::ConstPtr test)
  {
    ROS_ASSERT(slicer_);
    ROS_ASSERT(!test->class_map_.getCommonNames().empty());
    
    if(frame_stats_.class_map_.size() == 0)
      frame_stats_ = PerfStats(test->class_map_.getCommonNames());
    if(track_stats_.class_map_.size() == 0)
      track_stats_ = PerfStats(test->class_map_.getCommonNames());

    Classification trackcl(VectorXf::Zero(test->class_map_.getCommonNames().size()));
    double num_frames = 0;
    for(int i = 0; i < test->labels_.rows(); ++i) {
      if(test->labels_(i) == -2)
	continue;
      
      Classification framecl = slicer_->classify(test->descriptors_.col(i)).response_;
      frame_stats_.incrementStats(test->labels_(i), framecl.response_);

      trackcl.response_ += framecl.response_;
      ++num_frames;
      if(test->track_end_flags_(i) == 1) {
	trackcl.response_ /= num_frames;
	track_stats_.incrementStats(test->labels_(i), trackcl.response_);
	trackcl.response_.setZero();
	num_frames = 0;
      }
    }
  }

  void Evaluator::saveResults(const std::string& path)
  {
    ROS_ASSERT(fs::is_directory(path));

    frame_stats_.save(path + "/frame_results.txt");
    frame_stats_.saveConfusionMatrix(path + "/frame_confusion_matrix.png");
    frame_stats_.saveConfusionMatrix(path + "/frame_confusion_matrix.pdf");
    frame_stats_.savePrecisionRecallCurve(path + "/frame_pr.png");
    frame_stats_.savePrecisionRecallCurve(path + "/frame_pr.pdf");

    track_stats_.save(path + "/track_results.txt");
    track_stats_.saveConfusionMatrix(path + "/track_confusion_matrix.png");
    track_stats_.saveConfusionMatrix(path + "/track_confusion_matrix.pdf");
    track_stats_.savePrecisionRecallCurve(path + "/track_pr.png");
    track_stats_.savePrecisionRecallCurve(path + "/track_pr.pdf");
  }

} // namespace odontomachus
