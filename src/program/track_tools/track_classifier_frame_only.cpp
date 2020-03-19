#include <track_manager.h>
#include <boost/filesystem.hpp>
#include <multibooster_support.h>
#include <track_descriptors.h>

using namespace std;
using boost::shared_ptr;
using namespace track_manager;
using namespace track_descriptors;
using namespace pipeline;
using namespace Eigen;

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

void labelTrack(ClassifierPipeline& cp, shared_ptr<Track> track)
{
  // -- Compute individual frame responses.
  vector<VectorXf> frame_responses = cp.classify(track);
  assert(!frame_responses.empty());
  
  // -- Compute the track response.
  VectorXf track_response = VectorXf::Zero(frame_responses[0].rows());
  for(size_t i = 0; i < frame_responses.size(); ++i)
    track_response += 2.0 * (frame_responses[i] - cp.multibooster_->prior_);
  track_response /= (double)frame_responses.size();
  track_response += 2.0 * cp.multibooster_->prior_;

  // -- Apply the label.
  int prediction = -1;
  track->label_ = "background";
  if(track_response.maxCoeff() > 0) { 
    track_response.maxCoeff(&prediction);
    track->label_ = cp.multibooster_->class_map_.toName(prediction);
  }

}

int main(int argc, char** argv) {

  if(argc != 4) {
    cout << "Usage: " << argv[0] << " FRAME_CLASSIFIER TRACK_MANAGER_TO_CLASSIFY OUTPUT_TRACK_MANAGER" << endl;
    return 1;
  }

  string frame_classifier_filename(argv[1]);
  string track_manager_filename(argv[2]);
  string output_filename(argv[3]);

  if(output_filename.find(".tm") != string::npos) {
    cout << "Saving result as labeled track manager in " << output_filename  << endl;
  }
  else {
    cout << "Output filename " << output_filename << " must be a .tm file." << endl;
    return 1;
  }
  //assert(!boost::filesystem::exists(output_filename));
    
  // -- Load things.
  MultiBooster frame_classifier(frame_classifier_filename);
  ClassifierPipeline cp(&frame_classifier, NUM_THREADS);
  TrackManager tm(track_manager_filename);

  // -- Classify all tracks and set the labels.
  for(size_t i = 0; i < tm.tracks_.size(); ++i) {
    cout << "Working on track " << i << " / " << tm.tracks_.size() << endl;
    labelTrack(cp, tm.tracks_[i]);
  }
  
  // -- Save new Track Mananger with labels.
  tm.save(output_filename);
  
  return 0;
}
