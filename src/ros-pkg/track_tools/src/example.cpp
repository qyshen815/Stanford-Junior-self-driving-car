#include <track_tools/track_manager.h>
#include <iomanip>

using namespace std;

int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << "Usage: example TRACK_MANAGER" << endl;
    cout << " where TRACK_MANAGER is a .tm file containing tracks." << endl;
    return 1;
  }

  TrackManager tm(argv[1]);
  cout << "Loaded " << tm.tracks_.size() << " tracks." << endl;
  cout << endl;
  
  for(size_t i = 0; i < 4; ++i)
    cout << "Track " << i << " has " << tm.tracks_[i]->segments_.size() << " segments." << endl;
  cout << endl;

  sensor_msgs::PointCloud& cloud = *tm.tracks_[0]->segments_[0]->cloud_;
  cout << "The first segment of the first track has " << cloud.get_points_size() << " points." << endl;
  
  pose_t& pose = tm.tracks_[0]->segments_[0]->robot_pose_;
  cout << "The robot was at " << pose.x << " " << pose.y << " " << pose.z << ", "
       << pose.roll << " " << pose.pitch << " " << pose.yaw << endl;
  cout << endl;
  
  cout << "Point #: x y z intensity" << endl;
  cout << fixed;
  for(size_t i = 0; i < 10; ++i) {
    cout << setprecision(2);
    cout << "Point " << i << ": "
	 << cloud.points[i].x << " "
	 << cloud.points[i].y << " "
	 << cloud.points[i].z << " "
	 << setprecision(0) // Intensities are integer values between 0 and 255, but sensor_msgs::PointCloud stores them as floats.
	 << cloud.channels[0].values[i] << endl;;
  }
  
  return 0;
}

    
