#include <track_manager.h>
#include <track_descriptors.h>
#include <multibooster_support.h>

using namespace pipeline;
using namespace std;
using boost::shared_ptr;
using namespace track_manager;
using namespace sensor_msgs;
using namespace track_descriptors;
using namespace Eigen;


MultiBoosterDataset* extractDatasetFromTrackManager(const TrackManager& tm) {
  NameMapping class_map(getClassNames());

  // -- Set up multibooster dataset with the class and descriptor names.
  TrackClassifierPipeline tdp(NULL);
  MultiBoosterDataset* mbd = new MultiBoosterDataset(getClassNames(), tdp.getDescriptorNames());

    // -- Get a vector of Object*.
  vector<Object*> objects;
  objects.reserve(tm.tracks_[0]->frames_.size() * tm.tracks_.size()); //Should be ordered with longest first.
  for(size_t i = 0; i < tm.tracks_.size(); ++i) {
    cout << "Working on track " << i << " of " << tm.tracks_.size() << endl;
    Track& track = *tm.tracks_[i];
    
    // -- Compute the descriptors and put into object form.
    Object* obj = tdp.computeMultiBoosterObject(tm.tracks_[i]);
    
    // -- Set the label.
    int label = -2;
    if(track.label_.compare("unlabeled") == 0)
      continue;
    else if(track.label_.compare("background") == 0)
      label = -1;
    else
      label = class_map.toId(track.label_);
    
    obj->label_ = label;

    // -- Append to objects.
    objects.push_back(obj);
  }
        
  // -- Add to MultiBoosterDataset and return.
  mbd->setObjs(objects);
  return mbd;
}

int main(int argc, char** argv) {
  if(argc < 3) {
    cout << "Usage: " << argv[0] << " TRACK_MANAGER OUTPUT_DATASET_FILENAME " << endl;
    return 1;
  }

  if(getenv("SRAND"))
    srand(atoi(getenv("SRAND")));
  
  string dataset_filename(argv[2]);
  cout << "Dataset will be saved to " << dataset_filename << endl;
  
  string track_manager_filename(argv[1]);
  cout << "Loading track manager " << track_manager_filename << endl;
  TrackManager tm(track_manager_filename);
  cout << "Loaded " << tm.tracks_.size() << " tracks." << endl;
  if(tm.tracks_.size() == 0) {
    cerr << "0 tracks.  Aborting." << endl;
    return 2;
  }

  MultiBoosterDataset* mbd = extractDatasetFromTrackManager(tm);
  cout << "Saving dataset to " << dataset_filename << endl;
  mbd->save(dataset_filename);
  delete mbd;
  
  return 0;
}  
