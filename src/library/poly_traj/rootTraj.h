#ifndef ROOTTRAJ_H_
#define ROOTTRAJ_H_

#include <vector>
#include <map>
#include <trajectory_points_interface.h>

#include "polyTraj.h"

class RootTraj: public PolyTraj {
    
public:
    std::vector<vlr::CurvePoint> foot_curve_points_; // sampled at the required positions
    std::map<double, vlr::CurvePoint> center_line_; // discrete representation of the center curve
    
    RootTraj( const std::map<double, vlr::CurvePoint>& center_line );
    void calculateRootTrajectory();
    void evalCenterlineAtS(const double& s_interpol, vlr::CurvePoint& cp_interpol) const;
    static void evalCenterlineAtS_static(const std::map<double, vlr::CurvePoint>& center_line,
            const double& s_interpol, vlr::CurvePoint& cp_interpol);
};


#endif // ROOTTRAJ_H_
