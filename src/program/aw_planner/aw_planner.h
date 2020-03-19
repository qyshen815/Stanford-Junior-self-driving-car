#ifndef DGC_PLANNER_H
#define DGC_PLANNER_H

#include <vector>


#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <applanix_interface.h>
#include <localize_interface.h>
#include <estop_interface.h>
#include <passat_interface.h>
#include <perception_interface.h>
#include <traffic_light_interface.h>
#include <hci_interface.h>
#include <lltransform.h>

#include <aw_ChsmPlanner.hpp>

#include <param_interface.h>
#include <planner_interface.h>

#include "circle_demo.h"
#include "static_map_demo.h"

namespace vlr {

#define AW_PLANNER_PUBLISH_CYCLE_TIME .2
#define MAX_REQUESTED_TRAFFIC_LIGHTS 10
#define AW_PLANNER_IPC_TIMEOUT_MS  0.1

class AWRoadPlanner
{
public:
  AWRoadPlanner();
  ~AWRoadPlanner();
  void initialize(int argc, char **argv);


  ChsmPlanner* chsm_planner_;

  int last_estop_state_;

  std::string rndf_filename_;
  std::string mdf_filename_;
  double planner_hz_;
  dgc::IpcInterface* ipc_;
  dgc::ParamInterface* pint_;

  // robot pose
  double lat_, lon_;
  dgc::ApplanixPose applanix_pose_;
  dgc::LocalizePose localize_pose_;
  dgc_pose_t pose_accuracy_;
  std::vector<dgc_pose_t> pose_history_;
  dgc::EstopStatus estop_;
  PerceptionStopZones stop_zones_;

  pthread_mutex_t estop_mutex_;
  pthread_mutex_t stop_zones_mutex_;
  pthread_mutex_t traffic_light_mutex_;
  pthread_mutex_t vehicle_state_mutex_;


  param_struct_t cmd_params_;

  bool show_gui_;
  bool received_applanix_pose_;
  bool received_localize_pose_;
  bool received_stop_zones_;
  bool received_estop_;
  bool received_traffic_light_states_;
  bool received_vehicle_state_;
  bool data_ready_to_publish_;

  double static_obstacle_map_size_x_;
  double static_obstacle_map_size_y_;
  double static_obstacle_map_resolution_;

  bool run_planner_cycle_;
  bool wait_for_vehicle_;

  TrajectoryPoints2D trajectory_;
  TrafficLightPoseList traffic_light_pose_list_;
  dgc::PassatState vehicle_state_;


      // demo related stuff
  bool demo_mode_;
  TrajectoryPoint2D demo_start_point_;
  bool show_static_map_demo_;
  bool show_fourway_stop1_demo_;
  bool show_circle_demo_;

  BaseDemo* demo_;

  static const double circle_demo_r_;
  static const double circle_demo_start_lat_;
  static const double circle_demo_start_lon_;
  static const std::string circle_demo_rndf_filename_;
  static const std::string circle_demo_mdf_filename_;

  static const double static_map_demo_start_lat_;
  static const double static_map_demo_start_lon_;
  static const double static_map_demo_start_yaw_;
  static const std::string static_map_demo_rndf_filename_;
  static const std::string static_map_demo_mdf_filename_;
  static const std::string static_map_demo_map_name_;

public: //needed for demo(s)
  void applanixHandler(dgc::ApplanixPose* applanix_pose);
  void localizePoseHandler(dgc::LocalizePose* localize_pose);

public:
  void start();
  void run();
  void publish();

  inline void sendEStopRun() {estop_run_requested_=true;}
  inline void sendEStopPause() {estop_pause_requested_=true;};
  inline void quitPlanner() {
    pthread_mutex_lock(&quit_planner_mutex_);
    quit_planner_ = true;
    pthread_cond_signal(&quit_planner_cv_);
    pthread_mutex_unlock(&quit_planner_mutex_);
  }

private:
//  void initializePose();
  void updatePose(dgc::ApplanixPose* applanix_pose, dgc::LocalizePose* localize_pose);

  void readParameters(dgc::ParamInterface &pint, int argc, char **argv);
  void stopZonesHandler();
  void estopHandler();
  void fsmRequestHandler();
  void perceptionHandler(PerceptionObstacles* obstacles);
  void trafficLightHandler(TrafficLightList* tll);
  void vehicleStateHandler();

  void publishTrajectory(const std::vector<TrajectoryPoint2D>& trajectory_points);
  void publishTrafficLightRequest();
  void publishEstopRequest();
  void publishEmergencyMessage();

  void obstacleMapInitialize(int x_size, int y_size, double resolution);
  void clearObstacleMap();
  void waitForVehicle();

private:
  bool estop_run_requested_;
  bool estop_pause_requested_;
  dgc::EstopSoftstop estop_cmd_;

  bool caught_exception_;
  std::string caught_exception_text_;

  dgc::HciAudio hci_message_;
  std::string hci_message_string_;

  pthread_attr_t def_thread_attr_;

  pthread_t planner_thread_id_;
  pthread_t gui_thread_id_;

  bool quit_planner_;
  pthread_mutex_t quit_planner_mutex_;
  pthread_cond_t quit_planner_cv_;

  pthread_mutex_t emergency_message_sent_mutex_;
  pthread_cond_t emergency_message_sent_cv_;

  pthread_mutex_t received_vehicle_state_mutex_;
  pthread_cond_t received_vehicle_state_cv_;
};

} // namespace vlr
#endif

