#ifndef DGC_VELO_SUPPORT_H
#define DGC_VELO_SUPPORT_H

#include <roadrunner.h>
#include <logio.h>
#include <velocore.h>
#include <velodyne_interface.h>
#include <applanix_interface.h>
#include <localize_interface.h>

namespace dgc {

#define      MAX_NUM_VELOSCANS	      5000

typedef struct {
  int scan_num;
  double smooth_x, smooth_y, smooth_z;
  double longitude, latitude, altitude;
  double v_east, v_north, v_up;
  double roll, pitch, yaw;
  double timestamp;
  double x_offset, y_offset;
} dgc_velodyne_index_pose;

typedef struct {
 public:
  off64_t file_offset;
  int spin_start_i;
  int num_scans;
  int num_poses;
  dgc_velodyne_index_pose *pose;
} dgc_velodyne_index_entry;

class dgc_velodyne_index {
 public:
  dgc_velodyne_index();
  ~dgc_velodyne_index();
  int load(char *filename);
  int save(char *filename);
  int num_spins;
  dgc_velodyne_index_entry *spin;
};

class dgc_velodyne_spin {
 public:
  dgc_velodyne_spin();
  dgc_velodyne_spin( const dgc_velodyne_spin &copy );
  ~dgc_velodyne_spin();
  int num_scans;
  dgc_velodyne_scan_p scans;

  void copy(const dgc_velodyne_spin &copy);

  void save(char *filename, double applanix_lat, double applanix_lon,
	    double applanix_altitude);
  int load(char *filename, double *applanix_lat, double *applanix_lon,
	   double *applanix_altitude);
  void load(dgc_velodyne_file_p velodyne, dgc_velodyne_config_p config,
	    dgc_velodyne_index *vindex, int which, double *applanix_lat,
	    double *applanix_lon, double *applanit_alt);
};

typedef void
(*spin_function)(dgc_velodyne_spin *spin, 
		 dgc_velodyne_config_p config,
		 dgc::ApplanixPose *applanix_pose);

int
read_applanix_message(dgc_FILE *log_fp, dgc::LineBuffer *line_buffer, 
          dgc::ApplanixPose *applanix_pose, dgc::LocalizePose *localize_pose);

int
read_applanix_message(dgc_FILE *log_fp, dgc::LineBuffer *line_buffer,
		      dgc::ApplanixPose *applanix_pose);

void
vlf_projector(char *velo_log_filename, char *ipc_log_filename,
	      char *cal_filename, char *int_filename, dgc_transform_t velodyne_offset,
	      spin_function f);

}

#endif
