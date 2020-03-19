#ifndef KEEP_VELOCITY_TRAJ_SET_H_
#define KEEP_VELOCITY_TRAJ_SET_H_

#include <set>

#include <trajectory_points_interface.h>
#include "polyTraj.h"
#include "rootTraj.h"

//------------------- Longitudinal trajectory set for velocity keeping --------
class KeepVelocityTrajSet {
    // velocity_params par_; TODO
public:
    vlr::velocity_params par_; //TODO
    std::multiset<RootTraj,PolyTraj::CompPolyTraj> set_data_;
    bool slowSpeed_;

    KeepVelocityTrajSet(vlr::velocity_params params);
    int generate(double t, vlr::movement_state start_state, double desired_velocity,
            const std::map<double, vlr::CurvePoint> &center_line, vlr::GenerationMode mode);

    void clear();

    inline const vlr::velocity_params& params() const {return par_;}
    inline void params(const vlr::velocity_params& params) {par_=params;}

private:
    double logLowerVelocityStep(double vel_ref, uint32_t step, double step_size);
};


#endif // KEEP_VELOCITY_TRAJ_SET_H_
