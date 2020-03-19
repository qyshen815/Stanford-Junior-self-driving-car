#include "pose.h"

namespace vlr {

Pose::Pose() : smooth_x_(0), smooth_y_(0), smooth_z_(0),
               offset_x_(0), offset_y_(0), offset_z_(0),
               yaw_(0), pitch_(0), roll_(0),
               v_(0), v_lat_(0), v_z_(0), a_(0), a_lat_(0), a_z_(0) {
}

Pose::Pose(double smooth_x, double smooth_y, double yaw) :
               smooth_x_(smooth_x), smooth_y_(smooth_y), smooth_z_(0),
               offset_x_(0), offset_y_(0), offset_z_(0),
               yaw_(yaw), pitch_(0), roll_(0),
               v_(0), v_lat_(0), v_z_(0), a_(0), a_lat_(0), a_z_(0) {
}

Pose::Pose(double smooth_x, double smooth_y, double offset_x, double offset_y, double yaw) :
               smooth_x_(smooth_x), smooth_y_(smooth_y), smooth_z_(0),
               offset_x_(offset_x), offset_y_(offset_y), offset_z_(0),
               yaw_(yaw), pitch_(0), roll_(0),
               v_(0), v_lat_(0), v_z_(0), a_(0), a_lat_(0), a_z_(0) {
}

Pose::Pose(double smooth_x, double smooth_y, double offset_x, double offset_y,
     double yaw, double pitch, double roll,  double v, double v_lat, double v_z,
     double a, double a_lat, double a_z) :
     smooth_x_(smooth_x), smooth_y_(smooth_y), smooth_z_(0),
     offset_x_(offset_x), offset_y_(offset_y), offset_z_(0),
     yaw_(yaw), pitch_(pitch), roll_(roll),
     v_(v), v_lat_(v_lat), v_z_(v_z), a_(a), a_lat_(a_lat), a_z_(a_z) {
}

Pose::~Pose() {

}

} // namespace vlr
