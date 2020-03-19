#include <stdio.h>
#include <float.h>
#include <iostream>

#include <aw_kogmo_math.h>

#include <vlrException.h>
#include "rootTraj.h"

using namespace vlr;

//RootTraj::RootTraj( ) {
//}

RootTraj::RootTraj( const std::map<double, vlr::CurvePoint>& center_line ) {
    center_line_ = center_line;
}

void RootTraj::calculateRootTrajectory() {

    if ( discrete_traj_points_.size() == 0 ) {
        vlr::Exception except("Cannot calc root trajectory as there are no discrete_traj_points_ sets!");
        throw except;
    }
    if ( discrete_traj_points_.size() == 0 ) {
        vlr::Exception except("Cannot calc root trajectory as there are no points in first set of the discrete_traj_points_!");
        throw except;
    }
    std::vector<trajectory_point_1D>::const_iterator it_traj_point_1D;
    for ( it_traj_point_1D = discrete_traj_points_[0].begin();
            it_traj_point_1D != discrete_traj_points_[0].end();
            it_traj_point_1D++ ) {
        CurvePoint cp_temp;
 
        try {
        	evalCenterlineAtS(it_traj_point_1D->x, cp_temp);
        }
        catch(vlr::Exception except) {
        	vlr::Exception except_( "Calculate root trajectory -> " + except.getErrorMessage());
        	throw except_;
        }
        foot_curve_points_.push_back(cp_temp);
    }
    return;
}

void RootTraj::evalCenterlineAtS(const double& s_interpol,
        CurvePoint& cp_interpol) const {
    evalCenterlineAtS_static(center_line_, s_interpol, cp_interpol);
    return;
}

void RootTraj::evalCenterlineAtS_static(const std::map<double, CurvePoint>& center_line,
            const double& s_interpol, CurvePoint& cp_interpol) {
        
                      // Find closest index      
    if ( center_line.size() == 0 ) {
      throw vlr::Exception("Center line doesn't have entries!");
    }
    if ( s_interpol < center_line.begin()->second.s ) {
      throw vlr::Exception("Center line too short at the beginning!");
    }
    if ( s_interpol > center_line.rbegin()->second.s ) {
      throw vlr::Exception("Center line too short at the end!");
    }
    std::map<double, CurvePoint>::const_iterator it_upper = center_line.lower_bound(s_interpol);
    if(it_upper == center_line.end()) {
    	throw vlr::Exception("Current s not contained in center_line");
    }

    const CurvePoint& cp2 = it_upper->second;
    double s2 = cp2.s;
    it_upper--;  // it_lower now
    const CurvePoint& cp1 = it_upper->second;
    double s1 = cp1.s;

    double lambda;
    if ( (( s2 - s1 ) > DBL_EPSILON ) || (( s2 - s1 ) < -DBL_EPSILON ) ) {
        lambda = ( s_interpol - s1 ) / ( s2 - s1 );
    }
    else {
		vlr::Exception except("Center line points too close for interpolation!");
		throw except;
    }

    // Linear interpolation between points
    double x_interpol = cp1.x + lambda *( cp2.x - cp1.x );
    double y_interpol = cp1.y + lambda *( cp2.y - cp1.y );

    double delta_theta = cp2.theta - cp1.theta;

    delta_theta = vlr::normalizeAngle(delta_theta);
    double theta_interpol = cp1.theta + lambda * delta_theta;
    theta_interpol = vlr::normalizeAngle(theta_interpol);
    double kappa_interpol = cp1.kappa + lambda *( cp2.kappa  - cp1.kappa );
    double kappa_prime_interpol = cp1.kappa_prime + lambda *( cp2.kappa_prime - cp1.kappa_prime );

    cp_interpol.x           = x_interpol;
    cp_interpol.y           = y_interpol;
    cp_interpol.theta       = theta_interpol;
    cp_interpol.kappa       = kappa_interpol;
    cp_interpol.kappa_prime = kappa_prime_interpol;
}

