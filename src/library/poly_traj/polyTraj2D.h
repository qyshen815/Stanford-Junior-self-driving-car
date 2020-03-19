#ifndef POLYTRAJ2D_H_
#define POLYTRAJ2D_H_

#include <vector>
#include <iostream>

#include "polyTraj.h"
#include "polyTraj2D.h"
#include "rootTraj.h"

#include <trajectory_points_interface.h>

class PolyTraj2D {
    vlr::GenerationMode generation_mode_;
     double  total_cost_;
     double  max_curvature_;
    double min_velocity_;
    double  max_distance_; // to center line
    double  max_angle_;
    double  max_a_lat_;
    double  max_a_lon_;
    double  min_a_lon_;

    // Debug info:
    int index_lat_;
    int index_lon_;

    vlr::TrajectoryPoint2D latLongTrajectories2GlobalCordinates(const vlr::trajectory_point_1D& lat_traj,
            const vlr::trajectory_point_1D& long_traj, const vlr::CurvePoint& foot_curve_point ) const;
    static void projectionCondition(const std::map<double, vlr::CurvePoint>& center_line, const double& s_foot,
        const double& x1, const double& x2, double& f);  

public:
    const PolyTraj* pt_traj_lat_ ;
    const RootTraj* pt_traj_root_ ;
    std::vector<vlr::TrajectoryPoint2D> trajectory2D_;
    PolyTraj2D( const std::map<double, vlr::CurvePoint>& center_line, const PolyTraj& traj_lat,
            const RootTraj& root_traj, const vlr::GenerationMode mode,
            std::vector<std::vector<vlr::trajectory_point_1D> >::const_iterator it_lat_sampled_trajectory,
            int& index_lat, int& index_lon);
    double getInitialJerk() const;
    void calc_extreme_values(const double t_delay);
    //void calc_max_curvature();
    //void calc_min_velocity();
    //void calc_max_dist_to_centerline();
    //void calc_max_angle_to_centerline();
    double get_max_curvature() const    {return max_curvature_;}
    double get_min_velocity() const     {return min_velocity_;}
    double get_max_dist_to_centerline() const     {return max_distance_;}
    double get_max_angle_to_centerline() const     {return max_angle_;}
    double get_max_lon_acceleration() const     {return max_a_lon_;}
    double get_min_lon_acceleration() const     {return min_a_lon_;}
    double get_max_lat_acceleration() const     {return max_a_lat_;}
    double get_total_cost()  const      {return total_cost_;}
    int	   get_index_lat()  const		{return index_lat_;}
    int	   get_index_lon()  const		{return index_lon_;}
    void set_total_cost(double cost)    {total_cost_ = cost;}				// for debug
    void calculateNextInitialStates(const double& t_to_next_step,
        vlr::movement_state& next_lat_state,
            vlr::movement_state& next_long_state) const;
    vlr::TrajectoryPoint2D evaluateTrajectoryAtT(const double& t) const;
    vlr::TrajectoryPoint2D calculateNextStartTrajectoryPoint(const double& t_next) const;
    void evalLatLongTrajectoriesTimeBased(const double& t_act, const double& delta_t,
        vlr::trajectory_point_1D& lat_state,
        vlr::trajectory_point_1D& long_state) const;
    void evalLatLongTrajectoriesArclengthBased(const double& t_act, 
            const double& delta_t, const double& delta_s,
            vlr::trajectory_point_1D& lat_state,
            vlr::trajectory_point_1D& long_state) const;
    
    static void globalCordinates2LatLong(const vlr::TrajectoryPoint2D& tp_2d,
        const std::map<double, vlr::CurvePoint>& center_line, const vlr::GenerationMode& generation_mode,
        vlr::movement_state& lat_state, vlr::movement_state& lon_state);
    
    void generateNewControlTrajectory(const double& t_current, const double& t_control_horizon,
                                                  const double& control_t_res, std::vector<vlr::TrajectoryPoint2D>& trajectory) const;

    class CompPolyTraj2D { // in order to sort trajectories
    public:
        bool operator()(const PolyTraj2D &pt1, const PolyTraj2D &pt2) {
            if(pt1.total_cost_ < pt2.total_cost_ )
                return true;
            else
                return false;
        }
    };
    friend std::ostream &operator<< (std::ostream &ostr, const PolyTraj2D &traj2d);
private:
};

#endif // POLYTRAJ2D_H_
