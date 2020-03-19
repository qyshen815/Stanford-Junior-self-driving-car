#include <sys/stat.h>
#include <iostream> 
#include <fstream>
#include <sstream>
#include <track_manager.h>
#include <math.h>

using namespace std;
using namespace track_manager;
using namespace sensor_msgs;

string usageString() {
  ostringstream oss;
  oss << "Usage: " << endl;
  oss << "  track_stats TRACK_MANAGER [TRACK_MANAGER ...]" << endl;
  oss << "  track_stats --track-length-histogram TRACK_MANAGER [TRACK_MANAGER ...]" << endl;
  return oss.str();
}

void computeCentroid(const PointCloud& pc, float* x, float* y, float* z) {
  *x = 0;
  *y = 0;
  *z = 0;
  for(size_t i=0; i<pc.get_points_size(); ++i) {
    *x += pc.points[i].x;
    *y += pc.points[i].y;
    *z += pc.points[i].z;
  }
  *x /= (double)pc.get_points_size();
  *y /= (double)pc.get_points_size();
  *z /= (double)pc.get_points_size();
}
    

float computeSpeed(const PointCloud& pc, const PointCloud& prev) {
  float x = 0;
  float y = 0;
  float z = 0;
  computeCentroid(pc, &x, &y, &z);

  float x_prev = 0;
  float y_prev = 0;
  float z_prev = 0;
  computeCentroid(prev, &x_prev, &y_prev, &z_prev);

  float speed = sqrt(pow(x - x_prev, 2) + pow(y - y_prev, 2) + pow(z - z_prev, 2));
  return speed;
}
  

void trackLengthHistogram(int argc, char** argv) {
  map<size_t, size_t> hist;
  size_t max_length = 0;
  for(int i=2; i<argc; ++i) { 
    // -- Load the Track Manager class.
    TrackManager tm(argv[i]);

    for(size_t j=0; j<tm.tracks_.size(); ++j) {
      Track& tr = *tm.tracks_[j];
      ++hist[tr.frames_.size()];
      if(tr.frames_.size() > max_length)
	max_length = tr.frames_.size();
    }
  }

  size_t cum = 0;
  for(size_t i = max_length; i > 0; --i) {
    cum += hist[i];
    cout << i << " " << hist[i] << " " << cum << endl;
  }
}


void outputStats(int argc, char** argv) {
  map<string, int> num_clouds;
  map<string, int> num_tracks;
  for(int i=1; i<argc; ++i) { 
    // -- Load the Track Manager class.
    cout << "Working on " << argv[i] << endl;
    TrackManager tm(argv[i]);

    for(size_t j=0; j<tm.tracks_.size(); ++j) {
      Track& tr = *tm.tracks_[j];
      num_clouds[tr.label_]+= tr.frames_.size();
      num_tracks[tr.label_]++;
    }
  }

  int total_tracks = 0;
  int total_clouds = 0;
  cout << endl << "Track Statistics: " << endl;
  for(map<string, int>::iterator it = num_tracks.begin(); it!=num_tracks.end(); ++it) {
    cout << it->first << " tracks: " << it->second << endl;
    total_tracks += it->second;
  }
  cout << "Total tracks: " << total_tracks << endl << endl;
  cout << "Cloud Statistics: " << endl;
  for(map<string, int>::iterator it = num_clouds.begin(); it!=num_clouds.end(); ++it) {
    cout << it->first << " clouds: " << it->second << endl;
    total_clouds += it->second;
  }
  cout << "Total clouds: " << total_clouds << endl << endl;
}

int main(int argc, char** argv) {

  if(argc > 2 && strcmp(argv[1], "--track-length-histogram") == 0)
    trackLengthHistogram(argc, argv);
  else if(argc > 1 && strcmp(argv[1], "--track-length-histogram") != 0)
    outputStats(argc, argv);
  else
    cout << usageString() << endl;

 
  return 0;
}      


