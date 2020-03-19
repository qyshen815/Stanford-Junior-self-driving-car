#ifndef POLY_TRAJ_H_
#define POLY_TRAJ_H_

#include <vector>

#include <trajectory_points_interface.h>

//------------------- Definition of a polynomial trajectory ---------------
class PolyTraj{
    std::vector<double> coeff_;
    double te;      // zero-based, ts := 0
    double horizon_;   // zero-based sampling horizon, time or arg-length based
    double x0_;
    //double x_at_sample_horizon_;
    //double t0;      // absolute time
    vlr::movement_state end_state; // between 5th order and 3rd order polynomial for fast calcualtion
    double total_cost;
    double jerk_integral;
    double a_min;
    double a_max;

    // Debug info:
    double cost_arg_, cost_jerk_, cost_deviation_;
    double arg_, jerk_, deviation_;

    //double eval_poly(int derivative, double t);
    int calc_end_state();
    //double eval_trajectory(int derivative, double t);
    int calc_min_max_acceleration_of_1D_traj();
    void sample(const double& t_current, double time_resolution);
    void calc_jerk_integral();
    bool operator()(PolyTraj pt1, PolyTraj pt2);

public:
    std::vector<std::vector<vlr::trajectory_point_1D> > discrete_traj_points_;
    
    PolyTraj() : coeff_(6) {}

    int generate_5th_order( double x0, double x0_dot, double x0_ddot,
            double x1, double x1_dot, double x1_ddot,
            double t_current, double t, double horizon);
    int generate_4th_order( double x0, double x0_dot, double x0_ddot,
            double x1_dot, double x1_ddot, 
            double t_current, double t, double horizon);
    int eval_trajectory(const double& t, std::vector<double>& eval) const;
    double get_te()                     {return te;}
    double get_dder_min()                  {return a_min;}
    double get_dder_max()                  {return a_max;}
    double get_jerk_integral()          {return jerk_integral;}
    double get_total_cost()  const      {return total_cost;}
    void set_total_cost(double cost)    {total_cost = cost;}
    void set_debug_info( const double& arg, const double& jerk, const double& deviation, const double& cost_arg, const double& cost_jerk, const double& cost_deviation);
    void get_debug_info( double& arg, double& jerk, double& deviation, double& cost_arg, double& cost_jerk, double& cost_deviation) const;
    vlr::movement_state get_end_state()      {return end_state;}

    void sampleArgumentEquidistant(const double& t_current, double time_resolution);
    void sampleArgumentSynchronized(const std::vector<std::vector<double> >& sample_ref );

    class CompPolyTraj { // in order to sort the trajectories
    public:
        bool operator()(const PolyTraj &pt1, const PolyTraj &pt2) {
            if(pt1.total_cost < pt2.total_cost )
                return true;
            else
                return false;
        }
    };
};


#endif // POLY_TRAJ_H_
