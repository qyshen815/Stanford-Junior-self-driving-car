#include <track_manager.h>
#include <boost/filesystem.hpp>
#include "multibooster_support.h"
#include <track_descriptors.h>

using namespace std;
using boost::shared_ptr;
using namespace track_manager;
using namespace pipeline;
using namespace track_descriptors;
using namespace Eigen;

#define NUM_THREADS getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1

void accumulateStats(TrackClassifierPipeline& tcp, TrackManager& tm, PerfStats* stats) {
  for(size_t i = 0; i < tm.tracks_.size(); ++i) {
    cout << "Working on track " << i << " / " << tm.tracks_.size() << endl;
    Track& tr = *tm.tracks_[i];
    VectorXf response = tcp.classify(tm.tracks_[i]);
    assert((response.array() == response.array()).all()); // Make sure there are no nans.
    
    int label;
    if(tr.label_.compare("unlabeled") == 0)
      continue; 
    if(tr.label_.compare("background") == 0)
      label = -1;
    else
      label = tcp.getClassMap().toId(tr.label_);

    stats->incrementStats(label, response);
  }
}

int main(int argc, char** argv) {

  if(argc < 4) {
    cout << "Usage: " << argv[0] << " TRACK_CLASSIFIER TRACK_MANAGER [TRACK_MANAGER ...] RESULTS" << endl;
    return 1;
  }

  if(getenv("SRAND"))
    srand(atoi(getenv("SRAND")));

  string track_classifier_filename(argv[1]);
  string output_filename(argv[argc - 1]);
    
  if(boost::filesystem::exists(output_filename)) {
    cout << output_filename << " already exists, aborting." << endl;
    return 2;
  }
  
  // -- Load and set up the classifier.
  cout << "Loading multibooster " << track_classifier_filename << endl;
  MultiBooster* mb = new MultiBooster(track_classifier_filename);
  if(getenv("WC_LIMIT"))
    mb->wc_limiter_ = atoi(getenv("WC_LIMIT"));

  TrackClassifierPipeline tcp(mb);

  mb->applyNewMappings(mb->class_map_, tcp.getDescriptorNames());
  cout << mb->status(false) << endl;

  // -- Evaluate the classifier on each .tm file.
  PerfStats stats(mb->class_map_);
  for(int i = 2; i < argc - 1; ++i) { 
    cout << "Loading track manager " << argv[i] << endl;
    TrackManager tm(argv[i]);
    cout << "Loaded " << tm.tracks_.size() << " tracks." << endl;
    if(tm.tracks_.size() == 0) {
      cerr << "0 tracks.  Skipping." << endl;
      continue;
    }

    accumulateStats(tcp, tm, &stats);
  }

  delete mb;
  cout << stats.statString() << endl;
  stats.save(output_filename);

  stats.saveConfusionMatrix(output_filename + ".pdf");
  stats.saveConfusionMatrix(output_filename + ".png");
  return 0;
}
 
