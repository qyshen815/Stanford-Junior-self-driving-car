#ifndef KEEPLANEANDVELOCITYSET2D_H_
#define KEEPLANEANDVELOCITYSET2D_H_

#include <set>

#include <trajectory_points_interface.h>
#include "polyTraj2D.h"
#include "keep_lane_traj_set.h"
#include "keepVelocityTrajSet.h"

class KeepLaneAndVelocitySet2D {
public:
    keep_lane_traj_set keep_lane_traj_set_;                      // lane keeping set
    KeepVelocityTrajSet  keep_velocity_traj_set_;                // velocity set
    std::multiset<PolyTraj2D,PolyTraj2D::CompPolyTraj2D> set_data_; // cost sorted overlay of both

    KeepLaneAndVelocitySet2D(const vlr::lanekeeping_params &par_kl,
            const vlr::velocity_params &par_vel, const vlr::PolyTraj2D_params &par_2D);   // sets only the lat/long params of new object
    int generate(const std::map<double, vlr::CurvePoint> &center_line, double t,
    		vlr::TrajectoryPoint2D start_trajectory_point, vlr::GenerationMode generation_mode, double desired_velocity); // generates in each step the new lat, long and combined trajectories

    inline const vlr::PolyTraj2D_params& params() const {return par_;}
    inline void params(const vlr::PolyTraj2D_params& params) {par_=params;}
    inline void params(const vlr::lanekeeping_params& keep_lane_params, const vlr::velocity_params& keep_velocity_params,
                       const vlr::PolyTraj2D_params& params) {
      keep_lane_traj_set_.params(keep_lane_params);
      keep_velocity_traj_set_.params(keep_velocity_params);
      par_=params;
    }

private:
    vlr::PolyTraj2D_params par_;
};

#endif // KEEPLANEANDVELOCITYSET2D_H_
