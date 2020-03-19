#include <gtest/gtest.h>
#include <track_manager.h>
#include <track_manager_cached_descriptors.h>
#include <multibooster_support.h>

using namespace sensor_msgs;
using boost::shared_ptr;
using namespace track_manager;
using namespace std;

TEST(Frame, serialization) {
  dgc_transform_t velodyne_offset;
  getRandomTransform(velodyne_offset);
  
  shared_ptr<Frame> fr = getRandomFrame(velodyne_offset);
  string filename("frame_serialization_test.fr");
  ofstream file;
  file.open(filename.c_str());
  fr->serialize(file);
  file.close();

  ifstream file2;
  file2.open(filename.c_str());
  Frame fr2(file2, velodyne_offset);
  file2.close();

//   cout << fr->serialization_version_ << " " << fr2.serialization_version_ << endl;
//   cout << fr->timestamp_ << " " << fr2.timestamp_ << endl;

  EXPECT_TRUE(fr2 == *fr);
}

TEST(Track, serialization) {
  shared_ptr<Track> tr = getRandomTrack();
  string filename("track_serialization_test.tr");
  ofstream file;
  file.open(filename.c_str());
  tr->serialize(file);
  file.close();

  ifstream file2;
  file2.open(filename.c_str());
  Track tr2(file2);
  file2.close();

  EXPECT_TRUE(tr2 == *tr);
}

TEST(TrackManager, serialization) {
  shared_ptr<TrackManager> tm = getRandomTrackManager();
  string filename("track_manager_serialization_test.tm");
  tm->save(filename);

  TrackManager tm2(filename);

  cout << tm->tracks_.size() << endl;
  cout << tm2.tracks_.size() << endl;
  EXPECT_TRUE(tm2 == *tm);
}

TEST(TrackManagerCachedDescriptors, serialization)
{
  // -- Create a tm, cache the descriptors, and reload.
  shared_ptr<TrackManager> tm = getRandomTrackManager();
  TrackManagerCachedDescriptors tmcd(*tm, getClassNames());
  string path = "cached_descriptors_test.tm.mbd";
  tmcd.save(path);
  TrackManagerCachedDescriptors tmcd2(path);
  cout << tmcd.status() << endl << tmcd2.status() << endl;
  
  // -- Make sure they're the same.
  EXPECT_TRUE(tmcd.tracks_.size() == tmcd2.tracks_.size());
  for(size_t i = 0; i < tmcd.tracks_.size(); ++i) {
    EXPECT_TRUE(tmcd.tracks_[i]->frame_descriptors_.size() == tmcd2.tracks_[i]->frame_descriptors_.size());
    for(size_t j = 0; j < tmcd.tracks_[i]->frame_descriptors_.size(); ++j) {
      EXPECT_TRUE(tmcd.tracks_[i]->frame_descriptors_[j]->equals(*tmcd2.tracks_[i]->frame_descriptors_[j]));
    }
  }
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
