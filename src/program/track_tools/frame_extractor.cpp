#include <track_manager.h>

using namespace std;
using namespace track_manager;
using boost::shared_ptr;
using namespace Eigen;


void extractFrames(int num_frames, vector<string> input_filenames, string output_filename) { 
  int num_frames_per_tm = num_frames / input_filenames.size();

  TrackManager singletons;
  for(size_t i = 0; i < input_filenames.size(); ++i) {
    TrackManager tm(input_filenames[i]);
    
    int j = 0;
    while(j != num_frames_per_tm) {
      int track_id = rand() % tm.tracks_.size();
      Track& tr = *tm.tracks_[track_id];
      if(tr.label_.compare("background") == 0 ||
	 tr.label_.compare("unlabeled") == 0)
	continue;

      shared_ptr<Track> singleton(new Track(tr));
      singleton->frames_.clear();
      int frame_id = rand() % tr.frames_.size();
      singleton->frames_.push_back(tr.frames_[frame_id]);

      singletons.tracks_.push_back(singleton);
      ++j;
    }
  }

  singletons.save(output_filename);
}

int main(int argc, char** argv) {
  if(argc > 1) {
    int num_frames = 1000;
    string output_filename = "frames.tm";

    vector<string> input_filenames;
    for(int i = 1; i < argc; ++i)
      input_filenames.push_back(argv[i]);

    extractFrames(num_frames, input_filenames, output_filename);
  }
  else
    return 1;

  return 0;
}
