#include <track_manager_cached_descriptors.h>
#include <boost/filesystem.hpp>
#include <multibooster_support.h>
#include <eigen_extensions/eigen_extensions.h>
#include <track_descriptors.h>
#include "projector.h"

using namespace std;
using boost::shared_ptr;
using namespace track_manager;
using namespace track_descriptors;
using namespace pipeline;
using namespace Eigen;
namespace bfs = boost::filesystem;
namespace od = odontomachus;

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)
#define SEED (getenv("SEED") ? atol(getenv("SEED")) : 0)
#define NUM_PROJECTIONS (getenv("NUM_PROJECTIONS") ? atoi(getenv("NUM_PROJECTIONS")) : 200)

string usageString()
{
  ostringstream oss;
  oss << "Usage: label_and_project MULTIBOOSTER TM.MBD OD" << endl;
  oss << " Uses MULTIBOOSTER to classify TM.MBD, then projects in to OD." << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  if(argc != 4) {
    cout << usageString() << endl;
    return 0;
  }

  string mb_path = argv[1];
  string tmcd_path = argv[2];
  string od_path = argv[3];
  ROS_ASSERT(!bfs::exists(od_path));
  ROS_ASSERT(bfs::exists(mb_path));
  ROS_ASSERT(bfs::exists(tmcd_path));

  ROS_INFO_STREAM("Using " << mb_path << " to classify " << tmcd_path
		  << ", using " << NUM_PROJECTIONS << " projections and random seed of " << SEED
		  << ".  Saving to " << od_path);
  
  MultiBooster mb(mb_path, NUM_THREADS);
  TrackManagerCachedDescriptors tmcd(tmcd_path);
  ROS_INFO_STREAM(tmcd.status());
  ROS_ASSERT(mb.class_map_.compare(tmcd.class_map_));
  CachedClassifierPipeline ccp(&mb, NUM_THREADS);
  
  // -- Project into the OD with classifications.
  Projector proj(SEED, NUM_PROJECTIONS, *tmcd.tracks_[0]->frame_descriptors_[0]);
  od::Dataset::Ptr od(new od::Dataset());
  od->class_map_.addName("unlabeled");
  od->class_map_.addName("background");
  od->class_map_.setOffset(2);
  od->class_map_.addNames(mb.class_map_.getIdToNameMapping());
  od->descriptor_map_ = proj.generateNameMapping(tmcd.descriptor_map_);
  od->labels_ = VectorXi(tmcd.getTotalObjects());
  od->track_end_flags_ = VectorXi::Zero(tmcd.getTotalObjects());
  od->descriptors_ = MatrixXf(proj.getNumProjections(), tmcd.getTotalObjects());

  int idx = 0;
  double num_correct = 0;
  double num_with_labels = 0;
  for(size_t i = 0; i < tmcd.tracks_.size(); ++i) {
    if(i % (tmcd.tracks_.size() / 10) == 0)
      ROS_INFO_STREAM(i << "/" << tmcd.tracks_.size());
    
    TrackCachedDescriptors& tr = *tmcd.tracks_[i];
    int predicted_class_id;
    ccp.classify(tr.frame_descriptors_, &predicted_class_id);
    if(tr.label() != -2) {
      ++num_with_labels;
      if(tr.label() == predicted_class_id)
	++num_correct;
    }
    
    for(size_t j = 0; j < tr.frame_descriptors_.size(); ++j, ++idx) { 
      od->descriptors_.col(idx) = proj.project(*tr.frame_descriptors_[j]);
      od->labels_(idx) = predicted_class_id;
      if(j == tr.frame_descriptors_.size() - 1) 
	od->track_end_flags_(idx) = 1;
    }
  }

  ROS_INFO_STREAM("Accuracy on labeled tracks: " << num_correct / num_with_labels);
  od->assertConsistency();
  ROS_INFO_STREAM(od->status());
  od->save(od_path);

  return 0;
}
