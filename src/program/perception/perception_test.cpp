#include <roadrunner.h>
#include <vector>

#include "perception.h"
#include "box.h"

#include <gtest/gtest.h>

using namespace dgc;
using std::vector;

/* variables */

dgc_perception_map_cells_p     obstacles_s;
dgc_perception_map_cells_p     map_s;

std::vector<std::tr1::shared_ptr<TrackedObstacle> > obstacles_predicted;
std::vector<std::tr1::shared_ptr<TrackedObstacle> > obstacles_tracked;
std::vector<std::tr1::shared_ptr<Obstacle> >    obstacles_segmented;

char *imagery_root;
char *cal_filename = NULL;
GlsOverlay                    * gls = NULL;

/* tracker stuff */
grid_stat_t                     grid_stat;
dgc_grid_p                      grid;
dgc_grid_p                      terrain_grid;

dgc_perception_map_cell_p       default_map_cell     = NULL;
dgc_perception_map_cell_p       default_terrain_cell = NULL;

LocalizePose                    localize_pose = {0, 0.0, 0.0, "", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "" };

char                          * rndf_filename = NULL;
dgc_global_pose_t               global = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "10S"};
perception_settings_t           settings;

TEST(BoundingBox, Square) {
  vector<point3d_t> points;
  point3d_t pt;

  pt.x=1; pt.y=1;
  points.push_back(pt);

  pt.x=1; pt.y=2;
  points.push_back(pt);

  pt.x=2; pt.y=1;
  points.push_back(pt);
  pt.x=2; pt.y=2;
  points.push_back(pt);

  double x,y,yaw,width,length;
  bounding_box(&points, 0.0f, &x, &y, &yaw, &width, &length, 0.0f, 0.0f);
  EXPECT_DOUBLE_EQ(1.5, x);
  EXPECT_DOUBLE_EQ(1.5, y);
  EXPECT_DOUBLE_EQ(0.0, yaw);
  EXPECT_DOUBLE_EQ(1.0, width);
  EXPECT_DOUBLE_EQ(1.0, length);
}

TEST(BoundingBox, Rectangle) {
  vector<point3d_t> points;
  point3d_t pt;

  pt.x=1; pt.y=1;
  points.push_back(pt);

  pt.x=2; pt.y=1;
  points.push_back(pt);

  pt.x=1; pt.y=3;
  points.push_back(pt);

  pt.x=2; pt.y=3;
  points.push_back(pt);

  double x,y,yaw,width,length;
  bounding_box(&points, 0.0f, &x, &y, &yaw, &width, &length, 0.0f, 0.0f);

  EXPECT_DOUBLE_EQ(1.5, x);
  EXPECT_DOUBLE_EQ(2.0, y);
  EXPECT_DOUBLE_EQ(0.0, yaw);
  EXPECT_DOUBLE_EQ(2.0, width);
  EXPECT_DOUBLE_EQ(1.0, length);
}

TEST(BoundingBox, L) {
  vector<point3d_t> points;
  point3d_t pt;

  pt.x=1; pt.y=1;
  points.push_back(pt);

  pt.x=2; pt.y=1;
  points.push_back(pt);

  pt.x=1; pt.y=3;
  points.push_back(pt);

  double x,y,yaw,width,length;
  bounding_box(&points, 0.0f, &x, &y, &yaw, &width, &length, 0.0f, 0.0f);

  EXPECT_DOUBLE_EQ(1.5, x);
  EXPECT_DOUBLE_EQ(2.0, y);
  EXPECT_DOUBLE_EQ(0.0, yaw);
  EXPECT_DOUBLE_EQ(2.0, width);
  EXPECT_DOUBLE_EQ(1.0, length);
}

TEST(BoundingBox, NoisyL) {
  vector<point3d_t> points;
  point3d_t pt;

  pt.x=1; pt.y=1;
  points.push_back(pt);
  pt.x=1; pt.y=1.1;
  points.push_back(pt);
  pt.x=1.1; pt.y=1.1;
  points.push_back(pt);

  pt.x=2; pt.y=1;
  points.push_back(pt);
  pt.x=1.87; pt.y=1.01;
  points.push_back(pt);
  pt.x=2; pt.y=1;
  points.push_back(pt);

  pt.x=1; pt.y=3;
  points.push_back(pt);
  pt.x=1; pt.y=1.5;
  points.push_back(pt);
  pt.x=1; pt.y=2.5;
  points.push_back(pt);

  double x,y,yaw,width,length;
  bounding_box(&points, 0.0f, &x, &y, &yaw, &width, &length, 0.0f, 0.0f);

  EXPECT_DOUBLE_EQ(1.5, x);
  EXPECT_DOUBLE_EQ(2.0, y);
  EXPECT_DOUBLE_EQ(0.0, yaw);
  EXPECT_DOUBLE_EQ(2.0, width);
  EXPECT_DOUBLE_EQ(1.0, length);
}
