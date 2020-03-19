#include <online_learning/online_learning.h>
#include <track_manager_cached_descriptors.h>
#include <iostream>

using namespace std;
namespace bfs = boost::filesystem;
using namespace Eigen;
using namespace track_manager;
using namespace odontomachus;

string usageString()
{
  ostringstream oss;
  oss << "Usage: generate_raw_od TM.MBD OD" << endl;
  return oss.str();
}

NameMapping2 generateDescriptorMap(const TrackManagerCachedDescriptors& tmcd)
{
  NameMapping2 map;
  for(size_t i = 0; i < tmcd.descriptor_map_.size(); ++i) {
    VectorXf* vec = tmcd.tracks_[0]->frame_descriptors_[0]->descriptors_[i].vector;
    ROS_ASSERT(vec);
    for(int j = 0; j < vec->rows(); ++j) { 
      ostringstream oss;
      oss << tmcd.descriptor_map_.toName(i) << "::element_" << j;
      map.addName(oss.str());
    }
  }
  return map;
}

void concatenate(const Object& obj, VectorXf* output)
{
  int total = 0;
  for(size_t i = 0;  i < obj.descriptors_.size(); ++i) {
    ROS_ASSERT(obj.descriptors_[i].vector);
    total += obj.descriptors_[i].vector->rows();
  }
  ROS_ASSERT(total == output->rows());
  
  int idx = 0;
  for(size_t i = 0;  i < obj.descriptors_.size(); ++i) {
    int numel = obj.descriptors_[i].vector->rows();
    output->segment(idx, numel) = *obj.descriptors_[i].vector;
    idx += numel;
  }
}

int main(int argc, char** argv)
{
  if(argc != 3) {
    cout << usageString() << endl;
    return 0;
  }

  TrackManagerCachedDescriptors tmcd(argv[1]);
  cout << tmcd.status() << endl;

  // -- Set up the dataset.
  Dataset::Ptr data(new Dataset());
  data->class_map_.addName("unlabeled");
  data->class_map_.addName("background");
  data->class_map_.addNames(tmcd.class_map_.getIdToNameMapping());
  data->class_map_.setOffset(2);
  data->descriptor_map_ = generateDescriptorMap(tmcd);
  data->track_end_flags_ = VectorXi(tmcd.getTotalObjects());
  data->labels_ = VectorXi(tmcd.getTotalObjects());
  data->descriptors_ = MatrixXf(data->descriptor_map_.size(), tmcd.getTotalObjects());

  // -- Copy the data.
  int idx = 0;
  VectorXf buf(data->descriptor_map_.size());
  for(size_t i = 0; i < tmcd.tracks_.size(); ++i) {
    for(size_t j = 0; j < tmcd.tracks_[i]->frame_descriptors_.size(); ++j, ++idx) {
      data->labels_(idx) = tmcd.tracks_[i]->frame_descriptors_[j]->label_;
      concatenate(*tmcd.tracks_[i]->frame_descriptors_[j], &buf);
      data->descriptors_.col(idx) = buf;
      if(j == tmcd.tracks_[i]->frame_descriptors_.size() - 1)
	data->track_end_flags_(idx) = 1;
      else
	data->track_end_flags_(idx) = 0;
    }
  }

  // -- Check for screwups, save the dataset.
  data->assertConsistency();
  cout << data->class_map_.status() << endl;
  cout << tmcd.class_map_.serialize() << endl;
  cout << data->status() << endl;

  int track_idx = 0;
  int frame_idx = 0;
  for(int i = 0; i < data->labels_.rows(); ++i) {
    Object& obj = *tmcd.tracks_[track_idx]->frame_descriptors_[frame_idx];
    ROS_ASSERT(data->labels_(i) == obj.label_);

    int row = 0;
    for(size_t j = 0; j < obj.descriptors_.size(); ++j)
      for(int k = 0; k < obj.descriptors_[j].vector->rows(); ++k, ++row)
	ROS_ASSERT(fabs(data->descriptors_(row, i) - obj.descriptors_[j].vector->coeffRef(k)) <= 1e-6);
    
    ++frame_idx;
    if(data->track_end_flags_(i) == 1) { 
      ++track_idx;
      frame_idx = 0;
    }
  }

  data->save(argv[2]);
  return 0;
}
 
