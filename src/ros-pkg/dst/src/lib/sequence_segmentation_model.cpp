#include <dst/sequence_segmentation_model.h>

using namespace std;

namespace dst
{

  SequenceSegmentationModel::SequenceSegmentationModel()
  {
  }

  SequenceSegmentationModel::SequenceSegmentationModel(KinectSequence::ConstPtr seq) :
    seq_(seq)
  {
    figure_masks_.resize(seq_->images_.size());
    seed_masks_.resize(seq_->images_.size());

    for(size_t i = 0; i < seq_->images_.size(); ++i) {
      figure_masks_[i] = cv::Mat1b(seq_->images_[i].size(), 127);
      seed_masks_[i] = cv::Mat1b(seq_->images_[i].size(), 127);
    }
  }

  void SequenceSegmentationModel::save(const std::string& filename) const
  {
    ROS_FATAL_STREAM("Not yet implemented.");
  }
  
  void SequenceSegmentationModel::load(const std::string& filename)
  {
    ROS_FATAL_STREAM("Not yet implemented.");
  }
  
  void SequenceSegmentationModel::serialize(std::ostream& out) const
  {
    ROS_FATAL_STREAM("Use save instead.");
  }
  
  void SequenceSegmentationModel::deserialize(std::istream& in)
  {
    ROS_FATAL_STREAM("Use load instead.");
  }

  size_t SequenceSegmentationModel::size() const
  {
    ROS_ASSERT(seq_->images_.size() == seq_->pointclouds_.size());
    ROS_ASSERT(seq_->images_.size() == seed_masks_.size());
    ROS_ASSERT(seq_->images_.size() == figure_masks_.size());
    
    return seq_->images_.size();
  }


  
} // namespace dst
