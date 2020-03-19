/* This program doesn't work after the introduction of Frame.  Probably don't need it anyway... */

#include <track_manager.h>

using namespace std;
using namespace track_manager;
using namespace sensor_msgs;
using boost::shared_ptr;

void contrivify(Track* track, const TrackManager& tm, int num_duplicates)
{
  Track& tr = *track;
  
  // -- Get a random track with different label.
  int idx;
  while(true) {
    idx = rand() % tm.tracks_.size();
    if(tm.tracks_[idx]->label_.compare(tr.label_) != 0 &&
       tm.tracks_[idx]->label_.compare("unlabeled") != 0)
      break;
  }

  // -- Get a random cloud, timestamp, and velo center from that track.
  int idx2 = rand() % tm.tracks_[idx]->frames_.size();
  shared_ptr<PointCloud> cloud = tm.tracks_[idx]->frames_[idx2]->cloud_;
  double timestamp = tm.tracks_[idx]->frames_[idx2]->timestamp_;
  vector<float> center = tm.tracks_[idx]->velodyne_centers_[idx2];

  // -- Prepend.
  size_t new_size = (size_t)num_duplicates + tr.frames_.size();
  vector<double> timestamps(new_size);
  vector< vector<float> > centers(new_size);
  vector< shared_ptr<PointCloud> > clouds(new_size);
  for(size_t i = 0; i < new_size; ++i) {
    if(i < (size_t)num_duplicates) {
      timestamps[i] = timestamp;
      centers[i] = center;
      clouds[i] = cloud;
    }
    else {
      timestamps[i] = tr.frames_[i - num_duplicates]->timestamp_;
      centers[i] = tr.velodyne_centers_[i - num_duplicates];
      clouds[i] = tr.frames_[i - num_duplicates]->cloud_;
    }
  }
  tr.timestamps_ = timestamps;
  tr.clouds_ = clouds;
  tr.velodyne_centers_ = centers;
    
}

int main(int argc, char** argv) {

  if(argc != 4) { 
    cout << "Usage: " << argv[0] << " TRACK_MANAGER NUM_DUPLICATES SAVENAME" << endl;
    return 0;
  }

  int num_duplicates = atoi(argv[2]);
  assert(num_duplicates > 0);
  cout << "Loading " << argv[1] << ", prepending " << num_duplicates << " duplicate clouds with different label to the front of each track."
       << "  Saving as " << argv[3] << endl;
  TrackManager tm(argv[1]);
  TrackManager tm2(argv[1]);

  for(size_t i = 0; i < tm.tracks_.size(); ++i)
    contrivify(tm.tracks_[i].get(), tm2, num_duplicates);

  tm.save(argv[3]);
  return 0;
}

  
    
    

