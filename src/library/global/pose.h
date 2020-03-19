#ifndef GLOBAL_POSE_H_
#define GLOBAL_POSE_H_

namespace vlr {

class Pose {
public:
  Pose();
//  Pose(double smooth_x, double smooth_y);
  Pose(double smooth_x, double smooth_y, double yaw);
  Pose(double smooth_x, double smooth_y, double offset_x, double offset_y, double yaw);
  Pose(double smooth_x, double smooth_y, double offset_x, double offset_y,
       double yaw, double pitch, double roll,  double v, double v_lat, double v_z,
       double a, double a_lat, double a_z);
//  Pose(double smooth_x, double smooth_y, double smooth_z);

  virtual ~Pose();

  inline double x() const {return smooth_x_;}
  inline double& x() {return smooth_x_;}
  inline double y() const {return smooth_y_;}
  inline double& y() {return smooth_y_;}
  inline double z() const {return smooth_z_;}
  inline double& z() {return smooth_z_;}
  inline double utmX() const {return smooth_x_ + offset_x_;}
  inline double utmY() const {return smooth_y_ + offset_y_;}
  inline double offsetX() const {return offset_x_;}
  inline double& offsetX() {return offset_x_;}
  inline double offsetY() const {return offset_y_;}
  inline double& offsetY() {return offset_y_;}
  inline double offsetZ() const {return offset_z_;}
  inline double& offsetZ() {return offset_z_;}
  inline double yaw() const {return yaw_;}
  inline double& yaw() {return yaw_;}
  inline double pitch() const {return pitch_;}
  inline double& pitch() {return pitch_;}
  inline double roll() const {return roll_;}
  inline double& roll() {return roll_;}
  inline double v() const {return v_;}
  inline double& v() {return v_;}
  inline double vLateral() const {return v_lat_;}
  inline double& vLateral() {return v_lat_;}
  inline double vZ() const {return v_z_;}
  inline double& vZ() {return v_z_;}
  inline double a() const {return a_;}
  inline double& a() {return a_;}
  inline double aLateral() const {return a_lat_;}
  inline double& aLateral() {return a_lat_;}
  inline double aZ() const {return a_z_;}
  inline double& aZ() {return a_z_;}

protected:
  double smooth_x_, smooth_y_, smooth_z_;
  double offset_x_, offset_y_, offset_z_;
  double yaw_, pitch_, roll_;
  double v_, v_lat_, v_z_;
  double a_, a_lat_, a_z_;
};

} // namespace vlr
#endif
