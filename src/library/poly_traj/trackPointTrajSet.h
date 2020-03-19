/*
 * trackPointTrajSet.h
 *
 *  Created on: Sep 21, 2009
 *      Author: moritzwerling
 */

#ifndef TRACKPOINTTRAJSET_H_
#define TRACKPOINTTRAJSET_H_

#include <set>
#include <trajectory_points_interface.h>

#include "rootTraj.h"

class TrackPointTrajSet {
    // tracking_params par_; TODO
public:
	vlr::tracking_params par_; //TODO
  std::multiset<RootTraj,PolyTraj::CompPolyTraj> set_data_;
    //bool slowSpeed_;

    TrackPointTrajSet(vlr::tracking_params params);
    int generate(double t, vlr::movement_state start_state,
    		double desired_position, double desired_velocity, double desire_acceleration,
            const std::map<double, vlr::CurvePoint> &center_line, vlr::GenerationMode mode);
    void clear();

    inline const vlr::tracking_params& params() const {return par_;}
    inline void params(const vlr::tracking_params& params) {par_=params;}
};

#endif /* TRACKPOINTTRAJSET_H_ */
