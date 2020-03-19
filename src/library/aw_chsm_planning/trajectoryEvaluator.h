#ifndef TRAJECTORYEVALUATOR_H_
#define TRAJECTORYEVALUATOR_H_

#include <global.h>
#include <passat_constants.h>
#include <vlrException.h>
#include <trajectory_points_interface.h>
#include <poly_traj.h>

#include "obstaclePrediction.h"
#include "collisionCheck.h"
#include "curveSmoother.h"

namespace vlr {

class TrajectoryEvaluator {
 public:
  typedef enum {TRAJ_MODE_VELOCITY, TRAJ_MODE_FOLLOW, TRAJ_MODE_STOP} traj_mode_t;

class parameters {
  public:
  parameters();
  virtual ~parameters();

  double weight;  // balance between longitudinal/lateral trajectories
  double time_sample_res;
  double checked_horizon;
  double a_min, a_max;
  double control_t_horizon; // could be smaller than checked_horizon
  double control_t_res;
  velocity_params velocity_keeping;
  lanekeeping_params lane_keeping;
  tracking_params stop;
  tracking_params following;
  PolyTraj2D_params ptraj_2d;
  double t_to_next_step;
//  double reinit_orientation_thresh;
  double reinit_dist_thresh;
  double no_movement_reinit_timeout;
  double reinit_tangential_error_rate_thresh;
  double reinit_normal_error_rate_thresh;

    // Collision check params
  static const double passat_length = DGC_PASSAT_LENGTH;
  static const double passat_width = DGC_PASSAT_WIDTH;
  static const double passat_offset = DGC_PASSAT_IMU_TO_R_BUMPER + 1.0;

  static const double safety_length = DGC_PASSAT_LENGTH + 1.0;
  static const double no_safety_length = DGC_PASSAT_LENGTH;
  static const double safety_width = DGC_PASSAT_WIDTH + 1.0;
  static const double no_safety_width = DGC_PASSAT_WIDTH;

  double pull_away_time; // smaller than t_horizon!

  // width/length_safety    .. includes desired distance to other obstacles
  // width/length_no_safety   .. describes car with a fairly small safety margin in order be robust to noise of obstacle position
  // pull_away_time     .. time to pull away from a car that got too close

  double generation_velocity_threshold;

};

TrajectoryEvaluator(pthread_mutex_t& obstacle_map_mutex, pthread_mutex_t& dyn_obstacle_mutex);
 virtual ~TrajectoryEvaluator();

 inline const parameters& params() {return params_;}
 void setParams(parameters& params);

 void SetStaticObstacleMap(uint8_t* map, double width, double height, double res, double cx, double cy, double timestamp);
// void SetStaticObstacleMap(const uint8_t* map, double width, double height, double res, double cx, double cy, double timestamp);

 void makeTrajectory(const std::map<double, CurvePoint>& center_line,
     const std::map<int, Vehicle>& predicted_obstacles, double desired_v, double desired_a,
		    double stop_s, double s_lv, double s_dot_lv, double s_ddot_lv, const Pose& current_pose,
        double current_yaw_rate,
		    std::vector<TrajectoryPoint2D>& trajectory);//, std::vector<Frame>& frames);

 inline void reinitialize() {reinit_ = true;}

 inline const std::vector<TrajectoryPoint2D>& bestTrajectory() {return control_traj_;}

 inline traj_mode_t trajectoryMode() const {return active_traj_mode_;}
 inline const KeepLaneAndVelocitySet2D& getLVSet2D() const {return lane_and_velocity_set_;}
 inline const KeepLaneAndTrackPointSet2D& stopSet2D() const {return lane_and_stop_set_;}
 inline const KeepLaneAndTrackPointSet2D& getFollowingSet2D() const {return lane_and_follow_set_;}

   // This is the point our car was supposed to be at in the last planning cycle
 void getCurrentTrajectoryPoint(double t, TrajectoryPoint2D& current_tp);

private:
 bool checkReinit(const Pose& current_pose, double current_yaw_rate,  bool stop_in_sight,
                  bool lead_vehicle_in_sight, TrajectoryPoint2D& pred_state);

 void calculateNextStartTrajectoryPointReinit(double t0, double dt, const Pose& current_pose, double current_yaw_rate, TrajectoryPoint2D& pred_pose);
 bool checkTrajectoriesForCollision(const std::multiset<PolyTraj2D, PolyTraj2D::CompPolyTraj2D>& traj_set, const std::map<int, Vehicle>& predicted_obstacles,
                                    const std::string set_name, std::multiset<PolyTraj2D>::const_iterator& best_traj,
                                    double& longest_time_to_collision);

 bool checkCollisionOfTrajectoriesDynamic(const std::vector<TrajectoryPoint2D>& trajectory,
           const std::map<int, Vehicle>& obstacle_predictions, double& collision_time );

 bool checkCollisionVehicles(MovingBox& our_car_mb, const MovingBox& other_car_mb);

 TrjOccupancyState_t checkCollisionStatic(const std::vector<TrajectoryPoint2D>& trajectory);

 inline bool checkCollisionCircles(const Circle& c1, const Circle& c2) {

   double rs = c1.r + c2.r;
   double dx = c1.x - c2.x;
   double dy = c1.y - c2.y;

   return (rs*rs >= dx*dx + dy*dy);
 }

 double interpolateAngles(double theta1, double theta2, double alpha, double om_alpha);

private:
 parameters params_;

 vlr::CurveSmoother cs_;

 std::multiset<PolyTraj2D>::const_iterator it_best_;
 std::multiset<PolyTraj2D>::const_iterator active_traj_;

 std::vector<TrajectoryPoint2D> control_traj_;
 traj_mode_t active_traj_mode_;

 KeepLaneAndVelocitySet2D lane_and_velocity_set_;
 KeepLaneAndTrackPointSet2D lane_and_stop_set_;
 KeepLaneAndTrackPointSet2D lane_and_follow_set_;

 bool reinit_;

 TrajectoryPoint2D start_trajectory_point_;
 TrajectoryPoint2D current_trajectory_point_;

   // variables for collision check
 uint8_t* obstacle_map_;
 int32_t obstacle_map_width_;
 int32_t obstacle_map_height_;
 double obstacle_map_res_;
 double obstacle_map_cx_;
 double obstacle_map_cy_;
 double obstacle_map_timestamp_;
 pthread_mutex_t& obstacle_map_mutex_;
 pthread_mutex_t& dyn_obstacle_mutex_;
 };

} // namespace vlr

#endif // TRAJECTORYEVALUATOR_H_
