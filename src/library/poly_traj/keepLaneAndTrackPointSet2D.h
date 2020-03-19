/*
 * keepLaneAndTrackPointSet2D.h
 *
 *  Created on: Sep 21, 2009
 *      Author: moritzwerling
 */

#ifndef KEEPLANEANDTRACKPOINTSET2D_H_
#define KEEPLANEANDTRACKPOINTSET2D_H_

#include <float.h>
#include <set>

#include <trajectory_points_interface.h>

#include "polyTraj2D.h"
#include "keep_lane_traj_set.h"
#include "trackPointTrajSet.h"

class KeepLaneAndTrackPointSet2D {
 public:
    keep_lane_traj_set keep_lane_traj_set_;                  		// lane set
    TrackPointTrajSet  track_point_traj_set_;                		// point tracking set
    std::multiset<PolyTraj2D,PolyTraj2D::CompPolyTraj2D> set_data_; // cost sorted overlay of both

    KeepLaneAndTrackPointSet2D(const vlr::lanekeeping_params &par_kl,
            const vlr::tracking_params &par_track, const vlr::PolyTraj2D_params &par_2D);   // sets only the lat/long params of new object
    int generate(const std::map<double, vlr::CurvePoint> &center_line, double t,
    		vlr::TrajectoryPoint2D start_trajectory_point, vlr::GenerationMode generation_mode,
    		double desired_position, double desired_velocity, double desired_acceleration, bool no_check=false); // generates in each step the new lat, long and combined trajectories

    inline const vlr::PolyTraj2D_params& params() const {return par_;}
    inline void params(const vlr::PolyTraj2D_params& params) {par_=params;}
    inline void params(const vlr::lanekeeping_params& keep_lane_params, const vlr::tracking_params& tracking_params,
                       const vlr::PolyTraj2D_params& params) {
      keep_lane_traj_set_.params(keep_lane_params);
      track_point_traj_set_.params(tracking_params);
      par_=params;
    }

 private:
    vlr::PolyTraj2D_params par_;
};

class LeadingVehicle2Target {
	// Params
	double tau_;
	double d0_;
	double vel_thr_;
	double delta_s_thr_;

	// Last stop point
	double last_stop_s_;

public:
	LeadingVehicle2Target() : tau_ (1.), d0_ (5.), vel_thr_ (1.), delta_s_thr_(2.), last_stop_s_ (-DBL_MAX) {
	}
void calc(const double& s_lv, const double& s_dot_lv, const double& s_ddot_lv,
		double& s_target, double& s_dot_target, double& s_ddot_target);
};


#endif /* KEEPLANEANDTRACKPOINTSET2D_H_ */
