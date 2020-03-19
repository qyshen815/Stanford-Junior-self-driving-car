#include <track_descriptors.h>
#include <gtest/gtest.h>
//#include <multibooster_support.h>

using namespace std;
using namespace track_descriptors;
using namespace track_manager;

TEST(TrackDescriptors, MaxVel) {
  TrackManager tm("test.tm");
  TrackClassifierPipeline tdp(true);

  Object* obj;

  obj = tdp.computeMultiBoosterObject(tm.tracks_[1]); // About 10m/s
  cout << obj->status(vector<string>(), tdp.getDescriptorNames()) << endl;
  delete obj;

  obj = tdp.computeMultiBoosterObject(tm.tracks_[0]); // Stationary
  cout << obj->status(vector<string>(), tdp.getDescriptorNames()) << endl;
  delete obj;


  obj = tdp.computeMultiBoosterObject(tm.tracks_[237]); // Short track, moving.
  cout << obj->status(vector<string>(), tdp.getDescriptorNames()) << endl;
  delete obj;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
