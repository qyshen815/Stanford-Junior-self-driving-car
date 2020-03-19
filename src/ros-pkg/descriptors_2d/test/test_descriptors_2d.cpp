#include <descriptors_2d/descriptors_2d.h>
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <math.h>
#include <descriptors_2d/test_descriptors_2d.h>

using namespace std;
using namespace cv;

TEST(descriptors, SuperpixelColorHistogram) {
  SuperpixelColorHistogram desc(20, 0.5, 10);
  string name = "sch.results";
  EXPECT_TRUE(descriptorTest(&desc, name));
}

TEST(descriptors, Hog) {
  HogWrapper desc;
  string name = "hog.results";
  EXPECT_TRUE(descriptorTest(&desc, name));
}

TEST(descriptors, SURF) {
  SurfWrapper desc;
  string name = "surf.results";
  EXPECT_TRUE(descriptorTest(&desc, name));
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
