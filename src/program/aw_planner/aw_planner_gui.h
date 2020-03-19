#ifndef AW_PLANNER_GUI_H
#define AW_PLANNER_GUI_H

#include <rndfgl.h>
#include <vehiclemodels.h>

#include "aw_planner.h"

namespace vlr {

#define MAX_SCAN_DISTANCE_OBSTACLE  100.0

// display
#define CB_QUIT                   1
#define CB_REFRESH                2
#define CB_KTURN                  3
#define CB_ACTIVATE               4
#define CB_PAUSE                  5
#define CB_UPDATE_PLANNER_PARAMS  6
#define CB_UPDATE_TRAJECTORY_PARAMS 7
class AWRoadPlannerGUI {
public:
  ~AWRoadPlannerGUI();

  static AWRoadPlannerGUI* instance(AWRoadPlanner& awp, int argc, char** argv);
  static AWRoadPlannerGUI* instance();

  static inline void displayCBWrapper() {
    AWRoadPlannerGUI::instance()->display();
  }

  static inline void gluiControlCBWrapper(int control) {
    AWRoadPlannerGUI::instance()->gluiControlCallback(control);
  }

  static inline void keyboardCBWrapper(unsigned char key, int, int) {
    AWRoadPlannerGUI::instance()->keyboard(key);
  }

  void run();
  void keyboard(unsigned char key);
  void display();
  void gluiControlCallback(int control);

protected:
  AWRoadPlannerGUI(AWRoadPlanner& awp, int argc, char** argv);

private:
  void initializeGLUIControls();

  void drawObstacles(double center_x, double center_y);
  void drawPoseHistory(double center_x, double center_y);
  void drawTrajectories(Pose& pose);
  void drawBestTrajectory(Pose& pose);
  void drawCenterline(std::map<double, CurvePoint>& center_line, double center_x, double center_y);
  void drawCompleteCenterLine(const std::vector<CurvePoint>& mission_line_bez, const std::vector<CurvePoint>& mission_line, double center_x, double center_y);
  void drawTopology(double center_x, double center_y);
  void drawPoseAccuracy(dgc_pose_t robot_pose_accuracy);
  void drawDistanceCond(const double x, const double y, const double r);
  void drawEgoDistances(const double center_x, const double center_y, const double yaw, Topology* t);
  void drawDestinations(const double center_x, const double center_y);
  void drawVehicle();
  void updateObstacleMapTexture(const uint8_t* map_data, GLuint& texture, double& last_timestamp);
  void drawObstacleMap(GLuint texture, float r, float g, float b,
                        double cx, double cy, int32_t width, int32_t height, double res, double smooth_x, double smooth_y);
  void drawVehDistances(const double center_x, const double center_y, Topology* t);
  void drawIntersectionMergingPoint(const double center_x, const double center_y, Topology* t);
  void drawLaneChangeMergingPoints(const double center_x, const double center_y, Topology* t);
  void updateObstacleMapFromMessage(double& cx, double& cy, int32_t& width, int32_t& height, double& res);
  bool initFrameBufferObject(const uint32_t fb_width, const uint32_t fb_height,  GLuint& frame_buffer, GLuint& fb_texture);

private:
  AWRoadPlanner& awp_;

  static AWRoadPlannerGUI* instance_;


  static const float cmap_rb1_red_[256], cmap_rb2_red_[256];
  static const float cmap_rb1_green_[256], cmap_rb2_green_[256];
  static const float cmap_rb1_blue_[256], cmap_rb2_blue_[256];

GLUI* glui_;
GLUI_Rollout* planner_rollout_, *display_rollout_;

int show_topology_;
int show_rndf_;
int show_imagery_;
int show_vehicle_;
int show_pose_history_;
int show_curvepoints_;
int show_center_line_;
int show_complete_center_line_;
int show_trajectories_;
int show_best_trajectory_;
int show_complete_graph_;
int show_mission_graph_;
int show_matched_edge_;
int show_yellow_dot_;
int show_dynamic_objects_;
int show_complete_mission_graph_;
int show_obstacle_destinations_;
int show_obstacle_map_;
int show_configuration_space_;
int show_radius_;
int show_pose_accuracy_;
int show_ego_distances_;
int show_veh_distances_;
int show_intersection_merging_point_;
int show_lanchange_merging_points_;
int show_obstacle_predictions_;

ChsmPlanner::Parameters planner_params_;

  // imagery
//dgc_imagery_p imagery_;

dgc_passatwagonmodel_t* car_;
dgc_porsche_model* porsche_;
dgc_hummer_model* hummer_;
dgc_elise_model* elise_;

float max_velocity_, curvature_slowdown_factor_;
float urgency_;

GLuint configuration_space_texture_, obstacle_map_texture_;

GLUquadric* sphere_;
bool create_rndf_display_list_;

GLint rndf_display_list_;
double rndf_dl_origin_x_, rndf_dl_origin_y_;
coordinate_utm_t rndf_center_;
rndf_display_list rndf_display_list2_;
double last_raw_map_timestamp_, last_configuration_space_timestamp_;
double* smoothed_mission_points_buf_;  // buffer for OpenGL Bezier visualization
uint32_t smoothed_mission_points_buf_size_;

GLuint osm_circleconv_frame_buffer_;
GLuint osm_circleconv_texture_;
};

} // namespace vlr

#endif // AW_PLANNER_GUI_H
