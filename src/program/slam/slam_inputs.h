#ifndef DGC_SLAM_INPUTS_H
#define DGC_SLAM_INPUTS_H

#include <roadrunner.h>
#include <velocore.h>
#include <velo_support.h>
#include <gui3D.h>
#include <lltransform.h>
#include <vector>
#include <ANN.h>
#include "intensitycal.h"

struct Match {
  int tnum1, tnum2;
  int snum1, snum2;
  dgc_transform_t offset;
  int optimized;
};

class MatchList {
public:
  int num_matches(void) { return (int)match_.size(); }
  void AddMatch(Match m) { match_.push_back(m); }
  void Clear(void) { match_.clear(); }
  Match *match(int i) { return &(match_[i]); } 

private:
  std::vector <Match> match_;
};

struct SlamOrigin {
  double smooth_x, smooth_y, smooth_z;
  double utm_x, utm_y, utm_z;
  char utmzone[10];
};

class SlamLogfile {
 public:
  SlamLogfile();
  void SetVelodyneConfig(dgc_velodyne_config_p velodyne_config,
			 IntensityCalibration *intensity_calibration);

  int SetLogfileNames(char *log_filename, char *vlf_filename);
  int DiscoverLogfileNames(char *input_filename);

  void GeneratePathDL(SlamOrigin *origin);
  void DrawPath(void);

  void GenerateCorrectedPathDL(SlamOrigin *origin);
  void DrawCorrectedPath(void);

  void BuildPathKdtree(void);
  void ClosestPathPoint2D(double x, double y, int *min_s, double *min_dist);

  bool optimize(void) { return optimize_; }
  void set_optimize(bool optimize) { optimize_ = optimize; }

  dgc_velodyne_file_p fp(void) { return fp_; }
  dgc::dgc_velodyne_index *index(void) { return &index_; }
  dgc::dgc_velodyne_index *corrected_index(void) { return &corrected_index_; }
  dgc_velodyne_config_p velodyne_config(void) { return velodyne_config_; }
  IntensityCalibration *intensity_calibration(void) 
  { return intensity_calibration_; }
  pthread_mutex_t *mutex() { return &mutex_; }

  char *log_filename(void) { return log_filename_; }
  char *vlf_filename(void) { return vlf_filename_; }
  char *corrected_index_filename(void) { return corrected_vlf_index_filename_; }

 private:
  dgc_velodyne_file_p fp_;
  dgc::dgc_velodyne_index index_, corrected_index_;
  dgc_velodyne_config_p velodyne_config_;
  IntensityCalibration *intensity_calibration_;

  char *log_filename_;
  char *vlf_filename_;
  char *vlf_index_filename_;
  char *corrected_vlf_index_filename_;

  bool optimize_;

  GLint path_dl, corrected_path_dl;

  ANNpointArray kdtree_points_;
  ANNkd_tree *kdtree_;

  pthread_mutex_t mutex_;
};

class SlamInputs {
 public:
  int num_logs(void) { return (int)log_.size(); }
  SlamLogfile *log(int i) { return &(log_[i]); }
  void DiscoverInputFiles(int argc, char **argv,
			  dgc_velodyne_config_p velodyne_config,
			  IntensityCalibration *intensity_calibration);
  void LoadFromFile(char *filename, MatchList *matches,
		    dgc_velodyne_config_p velodyne_config,
		    IntensityCalibration *intensity_calibration);
  void BuildPathKdtrees(void);

 private:
  std::vector <SlamLogfile> log_;
};

inline void ConvertToUtm(double lat, double lon, double alt,
			 double *utm_x, double *utm_y, double *utm_z,
			 char *utmzone)
{
  vlr::latLongToUtm(lat, lon, utm_x, utm_y, utmzone);
  *utm_z = alt;
}

#endif
