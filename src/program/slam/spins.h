#ifndef DGC_SPINS_H
#define DGC_SPINS_H

#include <roadrunner.h>
#include <velocore.h>
#include <velo_support.h>
#include <textures.h>
#include <grid.h>
#include "slam_inputs.h"

struct MapCell {
  double intensity_sum;
  int intensity_count;
};

struct SpinPoint {
  double x, y, z;
  float i;
};

class SelectedSpin {
public:
  SelectedSpin();
  inline dgc::dgc_velodyne_spin *spin(void) { return &spin_; }
  inline double smooth_x(void) { return smooth_x_; }
  inline double smooth_y(void) { return smooth_y_; }
  inline double smooth_z(void) { return smooth_z_; }
  inline double utm_x(void) { return utm_x_; }
  inline double utm_y(void) { return utm_y_; }
  inline double utm_z(void) { return utm_z_; }
  inline char *utmzone(void) { return utmzone_; }
  inline bool valid(void) { return valid_; }
  void Select(SlamLogfile *log, int snum);
  SlamLogfile *log(void) { return log_; }
  bool texture_valid(void) { return texture_valid_; }

  void UpdateIntensityGrid(void);
  void DrawIntensityGrid(double origin_x, double origin_y, double origin_z,
			 double alpha);
  void DrawSpin(double origin_x, double origin_y, double origin_z, 
		dgc_transform_t t);

private:
  void FillRollingGrid(double travel_dist);

  bool valid_;
  int snum_;
  dgc::dgc_velodyne_spin spin_;
  std::vector <SpinPoint> point_;
  double smooth_x_, smooth_y_, smooth_z_;
  double utm_x_, utm_y_, utm_z_;
  char utmzone_[10];

  SlamLogfile *log_;

  dgc_grid_p grid_;
  unsigned char *texturedata_;
  vlr::dgc_gl_texture_t* texture_;
  bool texture_valid_;
};

#endif
