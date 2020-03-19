#include <eigen_extensions/random.h>
#include <gtest/gtest.h>

using namespace std;
using namespace Eigen;
using namespace eigen_extensions;

TEST(EigenExtensions, SparseRandom)
{
  SparseVector<double> vec;
  sampleSparseGaussianVector(100, 10, &vec);
  cout << vec.transpose() << endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
