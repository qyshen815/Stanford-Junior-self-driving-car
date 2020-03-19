#ifndef KEEP_LANE_TRAJ_SET_H_
#define KEEP_LANE_TRAJ_SET_H_

#include <set>

#include <trajectory_points_interface.h>

#include "polyTraj.h"
#include <vector>

//------------------- Lateral trajectory set for lane keeping --------
class keep_lane_traj_set {
    vlr::lanekeeping_params par;
public:
    std::multiset<PolyTraj,PolyTraj::CompPolyTraj> set_data; // contains the 1D trajectories

    keep_lane_traj_set(vlr::lanekeeping_params params);
    int generate(double t, double s, vlr::movement_state start_state, vlr::GenerationMode mode,
            double sampling_horizon, const std::vector<std::vector<double> >& sample_ref);
		// t 			.. time in the future we start generation from
		// start_state 	.. state in the future we start generation from
		// mode			.. normally time based, arc length based only for low speeds
		// sampling_hor .. temporal horizon we sample in for collision check
		// sample_ref	.. in order to combine the set with longitudinal traj sets, we sample according to their sample_ref

    void clear();

    inline const vlr::lanekeeping_params& params() const {return par;}
    inline void params(const vlr::lanekeeping_params& params) {par=params;}

    // get rid of data of previous cycle
};

#endif // KEEP_LANE_TRAJ_SET_H_
