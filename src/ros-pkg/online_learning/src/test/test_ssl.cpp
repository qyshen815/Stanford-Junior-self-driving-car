#include <online_learning/ssl.h>
#include <gtest/gtest.h>

using namespace std;
using namespace odontomachus;

TEST(EpochStatistics, Table)
{
  EpochStatistics stats(getDefaultClassMap());
  stats.addTrack(20, 1, 1);
  stats.addTrack(20, 1, 0);
  stats.addTrack(20, 0, 0);
  stats.addTrack(30, 0, 2);
  cout << stats << endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

