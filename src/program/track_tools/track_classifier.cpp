#include <track_manager.h>
#include <boost/filesystem.hpp>
#include <multibooster_support.h>
#include <eigen_extensions/eigen_extensions.h>
#include <track_descriptors.h>

using namespace std;
using boost::shared_ptr;
using namespace track_manager;
using namespace track_descriptors;
using namespace pipeline;
using namespace Eigen;

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

int main(int argc, char** argv) {

  if(argc != 6) {
    cout << "Usage: " << argv[0] << " FRAME_CLASSIFIER TRACK_CLASSIFIER WEIGHTS TRACK_MANAGER_TO_CLASSIFY OUTPUT_TRACK_MANAGER" << endl;
    return 1;
  }

  string frame_classifier_filename(argv[1]);
  string track_classifier_filename(argv[2]);
  string weights_filename(argv[3]);
  string track_manager_filename(argv[4]);
  string output_filename(argv[5]);

  if(output_filename.find(".tm") != string::npos) {
    cout << "Saving result as labeled track manager in " << output_filename  << endl;
  }
  else {
    cout << "Output filename " << output_filename << " must be a .tm file." << endl;
    return 1;
  }
  //assert(!boost::filesystem::exists(output_filename));
    
  // -- Load things.
  VectorXd weights;
  eigen_extensions::load(weights_filename, &weights);
  CombinedClassifierPipeline ccp(frame_classifier_filename,
				 track_classifier_filename,
				 weights,
				 NUM_THREADS);

  TrackManager tm(track_manager_filename);

  // -- Classify all tracks and set the labels.
  for(size_t i = 0; i < tm.tracks_.size(); ++i) {
    cout << "Working on track " << i << " / " << tm.tracks_.size() << endl;
    ccp.classify(tm.tracks_[i], &tm.tracks_[i]->label_);
  }
    
  // -- Save new Track Mananger with labels.
  tm.save(output_filename);
  
  return 0;
}
