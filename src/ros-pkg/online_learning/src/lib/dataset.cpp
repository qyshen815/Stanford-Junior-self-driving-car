#include <online_learning/online_learning.h>

using namespace Eigen;
using namespace std;

namespace odontomachus
{

  /************************************************************
   * Dataset
   ************************************************************/
  
  void Dataset::serialize(ostream& out) const
  {
    class_map_.serialize(out);
    descriptor_map_.serialize(out);
    eigen_extensions::serialize(labels_, out);
    eigen_extensions::serialize(track_end_flags_, out);
    eigen_extensions::serialize(descriptors_, out);
  }

  void Dataset::deserialize(istream& in)
  {
    class_map_.deserialize(in);
    descriptor_map_.deserialize(in);
    eigen_extensions::deserialize(in, &labels_);
    eigen_extensions::deserialize(in, &track_end_flags_);
    eigen_extensions::deserialize(in, &descriptors_);
  }

  std::string Dataset::status() const
  {
    map<int, int> frame_counts;
    map<int, int> track_counts;
    for(int i = 0; i < labels_.rows(); ++i) { 
      ++frame_counts[labels_(i)];
      if(track_end_flags_(i) == 1)
	++track_counts[labels_(i)];
    }
    
    ostringstream oss;
    oss << "=== Dataset status" << endl;
    oss << "Total tracks: " << track_end_flags_.sum() << endl;
    oss << "Total frames: " << descriptors_.cols() << endl;
    oss << "Instance dimensionality: " << descriptors_.rows() << endl;
    oss << endl;
    oss << "Track count: " << endl;
    map<int, int>::iterator it;
    for(it = track_counts.begin(); it != track_counts.end(); ++it)
      oss << class_map_.toName(it->first) << ": " << it->second << endl;
    oss << endl;
    oss << "Frame count: " << endl;
    for(it = frame_counts.begin(); it != frame_counts.end(); ++it)
      oss << class_map_.toName(it->first) << ": " << it->second << endl;

    return oss.str();
  }

  Dataset& Dataset::operator+=(const Dataset& other)
  {
    other.assertConsistency();
    ROS_ASSERT(class_map_ == other.class_map_ || class_map_.empty());
    if(other.labels_.rows() == 0)
      return *this;
    
    if (descriptor_map_ != other.descriptor_map_ && !descriptor_map_.empty()) {
      vector<string> here_but_not_there;
      vector<string> there_but_not_here;
      descriptor_map_.diff(other.descriptor_map_, &here_but_not_there, &there_but_not_here);

      cout << "here_but_not_there: " << endl;
      for(size_t i = 0; i < here_but_not_there.size(); ++i)
	cout << here_but_not_there[i] << endl;
      cout << "there_but_not_here: " << endl;
      for(size_t i = 0; i < there_but_not_here.size(); ++i)
	cout << there_but_not_here[i] << endl;

      ROS_ASSERT(0);
    }
      
    ROS_ASSERT(!other.descriptor_map_.empty());
    ROS_ASSERT(!other.class_map_.empty());
    
    if(descriptors_.cols() == 0) {
      if(this == &other)
	return *this;

      *this = other;
      return *this;
    }

    if(class_map_.empty())
      class_map_ = other.class_map_;
    if(descriptor_map_.empty())
      descriptor_map_ = other.descriptor_map_;
      
    ROS_FATAL_STREAM_COND(other.descriptors_.rows() != descriptors_.rows(),
			  "Attempted to add datasets with dimensionality " << descriptors_.rows() << " and " << other.descriptors_.rows());

    Eigen::MatrixXf new_descriptors(descriptors_.rows(), descriptors_.cols() + other.descriptors_.cols());
    new_descriptors.block(0, 0, descriptors_.rows(), descriptors_.cols()) = descriptors_;
    new_descriptors.block(0, descriptors_.cols(), other.descriptors_.rows(), other.descriptors_.cols()) = other.descriptors_;
    descriptors_ = new_descriptors;

    Eigen::VectorXi new_labels(labels_.rows() + other.labels_.rows());
    new_labels.head(labels_.rows()) = labels_;
    new_labels.tail(other.labels_.rows()) = other.labels_;
    labels_ = new_labels;

    Eigen::VectorXi new_track_end_flags(track_end_flags_.rows() + other.track_end_flags_.rows());
    new_track_end_flags.head(track_end_flags_.rows()) = track_end_flags_;
    new_track_end_flags.tail(other.track_end_flags_.rows()) = other.track_end_flags_;
    track_end_flags_ = new_track_end_flags;

    assertConsistency();
    return *this;
  }

  void Dataset::applyNameMapping(const NameMapping2& new_mapping, int id)
  {
    if(id == 0)
      NameMappable::applyNameMapping(class_map_, new_mapping, id);
    else if(id == 1)
      NameMappable::applyNameMapping(descriptor_map_, new_mapping, id);
    else
      ROS_FATAL_STREAM("No NameMapping2 with id " << id);
  }
  
  void Dataset::applyNameMapping(const NameTranslator2& translator,
				 const NameMapping2& new_mapping,
				 int id)
  {
    if(id == 0)
      applyClassMap(translator, new_mapping);
    else if(id == 1)
      applyDescriptorMap(translator, new_mapping);
    else
      ROS_FATAL_STREAM("No NameMapping2 with id " << id);
  }

  void Dataset::applyClassMap(const NameTranslator2& translator,
			      const NameMapping2& new_mapping)
  {
    ROS_FATAL("Not implemented.");
  }

  void Dataset::applyDescriptorMap(const NameTranslator2& translator,
				   const NameMapping2& new_mapping)
  {
    ROS_ASSERT(translator.newSize() <= translator.oldSize());
    MatrixXf descriptors(translator.newSize(), descriptors_.cols());
    for(int i = 0; i < descriptors_.rows(); ++i) { 
      if(translator.oldToNewId(i) == NameTranslator2::NO_ID)
	continue;
      descriptors.row(translator.oldToNewId(i)) = descriptors_.row(i);
    }
    descriptors_ = descriptors;
    descriptor_map_ = new_mapping;
  }

  void Dataset::assertConsistency() const
  {
    ROS_ASSERT(descriptors_.cols() == labels_.rows());
    ROS_ASSERT(labels_.rows() == track_end_flags_.rows());

    for(int i = 0; i < track_end_flags_.rows(); ++i) { 
      ROS_ASSERT(track_end_flags_(i) == 0 || track_end_flags_(i) == 1);
      if(!track_end_flags_(i))
	ROS_ASSERT(labels_(i) == labels_(i+1));
    }

    if(labels_.rows() == 0)
      return;

    int min_class_id = labels_.minCoeff();
    int max_class_id = labels_.maxCoeff();
    ROS_ASSERT(min_class_id >= -class_map_.getOffset());
    ROS_ASSERT(max_class_id <= (int)class_map_.size() - 1 - class_map_.getOffset());

    ROS_ASSERT(descriptor_map_.getOffset() == 0);
    ROS_ASSERT(descriptor_map_.size() == (size_t)descriptors_.rows());

    ROS_ASSERT(track_end_flags_(track_end_flags_.rows() - 1) == 1);
  }
  
  /************************************************************
   * Helper functions
   ************************************************************/

  NameMapping2 getDefaultClassMap()
  {
    NameMapping2 class_map;
    class_map.addName("unlabeled");
    class_map.addName("background");
    class_map.addName("car");
    class_map.addName("pedestrian");
    class_map.addName("bicyclist");
    class_map.setOffset(2); // background = -1, unlabeled = -2.
    return class_map;
  }

  NameMapping2 getStubDescriptorMap(int num_descriptors)
  {
    NameMapping2 dmap;
    for(int i = 0; i < num_descriptors; ++i) { 
      ostringstream oss;
      oss << "dspace" << i;
      dmap.addName(oss.str());
    }
    return dmap;
  }
  

  
} // namespace
