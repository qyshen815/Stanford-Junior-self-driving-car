#include <roadrunner.h>
#include <velocore.h>
#include "slam_inputs.h"

using namespace dgc;

SlamLogfile::SlamLogfile()
{
  path_dl = 0;
  corrected_path_dl = 0;
  pthread_mutex_init(&mutex_, NULL);
}

void SlamLogfile::SetVelodyneConfig(dgc_velodyne_config_p velodyne_config,
				    IntensityCalibration *intensity_calibration)
{
  velodyne_config_ = velodyne_config;
  intensity_calibration_ = intensity_calibration;
}

void AbsoluteFilename(char **filename)
{
  //char *new_filename = canonicalize_file_name(*filename);
  char *new_filename = realpath(*filename,NULL);

  if (new_filename == NULL) {
    dgc_die("Could not canonicalize filename %s\n", *filename);
  } else {
    free(*filename);
    *filename = new_filename;
  }
}

int SlamLogfile::SetLogfileNames(char *log_filename, char *vlf_filename)
{
  log_filename_ = strdup(log_filename);
  vlf_filename_ = strdup(vlf_filename);

  vlf_index_filename_ = new char[strlen(vlf_filename_) + 20];
  sprintf(vlf_index_filename_, "%s.index.gz", vlf_filename_);
  if (!dgc_file_exists(vlf_index_filename_)) {
    dgc_error("Could not find vlf index file for %s\n", vlf_filename_);
    return -1;
  }

  AbsoluteFilename(&log_filename_);
  AbsoluteFilename(&vlf_filename_);
  AbsoluteFilename(&vlf_index_filename_);

  corrected_vlf_index_filename_ = new char[strlen(vlf_filename_) + 30];
  sprintf(corrected_vlf_index_filename_, "%s.fixed-index.gz", vlf_filename_);
  fprintf(stderr, "corrected index filename = %s\n", corrected_vlf_index_filename_);

  fp_ = dgc_velodyne_open_file(vlf_filename_);
  if (fp_ == NULL) {
    dgc_error("Could not open velodyne file %s for reading.\n", 
	      vlf_filename_);
    return -1;
  }

  if (index_.load(vlf_index_filename_) < 0)
    dgc_die("Could not load vlf index %s\n", vlf_index_filename_);

  if (!dgc_file_exists(corrected_vlf_index_filename_)) {
    if (corrected_index_.load(vlf_index_filename_) < 0)
      dgc_die("Could not load vlf index %s\n", vlf_index_filename_);
  }
  else {
    if (corrected_index_.load(corrected_vlf_index_filename_) < 0)
      dgc_die("Could not load vlf index %s\n", corrected_vlf_index_filename_);
  }
  return 0;
}

int SlamLogfile::DiscoverLogfileNames(char *input_filename)
{
  if (dgc_complete_filename(input_filename, ".log.gz", &log_filename_) ||
      dgc_complete_filename(input_filename, ".log", &log_filename_)) {
    vlf_filename_ = 
      find_matching_logfile(log_filename_, ".vlf", 30);
    if (vlf_filename_ == NULL) {
      dgc_error("Could not find matching .vlf file for %s\n", log_filename_);
      return -1;
    }
    
    vlf_index_filename_ = new char[strlen(vlf_filename_) + 20];
    sprintf(vlf_index_filename_, "%s.index.gz", vlf_filename_);
    if (!dgc_file_exists(vlf_index_filename_)) {
      dgc_error("Could not find vlf index file for %s\n", vlf_filename_);
      return -1;
    }

    AbsoluteFilename(&log_filename_);
    AbsoluteFilename(&vlf_filename_);
    AbsoluteFilename(&vlf_index_filename_);

    corrected_vlf_index_filename_ = new char[strlen(vlf_filename_) + 30];
    sprintf(corrected_vlf_index_filename_, "%s.fixed-index.gz", vlf_filename_);
    fprintf(stderr, "corrected index filename = %s\n", corrected_vlf_index_filename_);

    fp_ = dgc_velodyne_open_file(vlf_filename_);
    if (fp_ == NULL) {
      dgc_error("Could not open velodyne file %s for reading.\n", 
		vlf_filename_);
      return -1;
    }

    if (index_.load(vlf_index_filename_) < 0)
      dgc_die("Could not load vlf index %s\n", vlf_index_filename_);

    if (!dgc_file_exists(corrected_vlf_index_filename_)) {
      if (corrected_index_.load(vlf_index_filename_) < 0)
	dgc_die("Could not load vlf index %s\n", vlf_index_filename_);
    }
    else {
      if (corrected_index_.load(corrected_vlf_index_filename_) < 0)
	dgc_die("Could not load vlf index %s\n", corrected_vlf_index_filename_);
    }

    return 0;
  }
  else {
    dgc_error("Invalid argument \"%s\"\n", input_filename);
    return -1;
  }
}

void SlamLogfile::BuildPathKdtree(void)
{
  char utmzone[10];
  double x, y, z;
  int j;
  
  kdtree_points_ = annAllocPts(index_.num_spins, 2);
  for (j = 0; j < index_.num_spins; j++) {
    ConvertToUtm(index_.spin[j].pose[0].latitude,
		 index_.spin[j].pose[0].longitude,
		 index_.spin[j].pose[0].altitude,
		 &x, &y, &z, utmzone);
    kdtree_points_[j][0] = x;
    kdtree_points_[j][1] = y;
  }
  kdtree_ = new ANNkd_tree(kdtree_points_, index_.num_spins, 2);
}

void SlamLogfile::ClosestPathPoint2D(double x, double y, int *min_s,
				     double *min_dist)
{
  ANNpoint queryPt = annAllocPt(2);
  ANNidxArray nn_index = new ANNidx[1];
  ANNdistArray nn_dist = new ANNdist[1];

  queryPt[0] = x;
  queryPt[1] = y;
  kdtree_->annkSearch(queryPt, 1, nn_index, nn_dist);
  *min_s = nn_index[0];
  *min_dist = sqrt(nn_dist[0]);
  delete queryPt;
  delete nn_index;
  delete nn_dist;  
}

void SlamLogfile::GeneratePathDL(SlamOrigin *origin)
{
  dgc_velodyne_index_pose *pose;
  char utmzone[10];
  double x, y, z;
  int j;

  if (path_dl != 0)
    glDeleteLists(path_dl, 1);
  path_dl = glGenLists(1);
  glNewList(path_dl, GL_COMPILE);

  glBegin(GL_QUAD_STRIP);
  for (j = 0; j < index_.num_spins; j++) {
    pose = &(index_.spin[j].pose[0]);
    ConvertToUtm(pose->latitude, pose->longitude, pose->altitude,
		 &x, &y, &z, utmzone);
    glVertex3f(x + cos(pose->yaw + dgc_d2r(90)) - origin->utm_x, 
	       y + sin(pose->yaw + dgc_d2r(90)) - origin->utm_y, 
	       z - origin->utm_z);
    glVertex3f(x + cos(pose->yaw - dgc_d2r(90)) - origin->utm_x, 
	       y + sin(pose->yaw - dgc_d2r(90)) - origin->utm_y,
	       z - origin->utm_z);
  }
  glEnd();

  glEndList();
}

void SlamLogfile::GenerateCorrectedPathDL(SlamOrigin *origin)
{
  dgc_velodyne_index_pose *pose;
  char utmzone[10];
  double x, y, z;
  int j;

  if (corrected_path_dl != 0)
    glDeleteLists(corrected_path_dl, 1);
  corrected_path_dl = glGenLists(1);
  glNewList(corrected_path_dl, GL_COMPILE);

  glBegin(GL_QUAD_STRIP);
  for (j = 0; j < corrected_index_.num_spins; j++) {
    pose = &(corrected_index_.spin[j].pose[0]);
    ConvertToUtm(pose->latitude, pose->longitude, pose->altitude,
		 &x, &y, &z, utmzone);
    glVertex3f(x + cos(pose->yaw + dgc_d2r(90)) - origin->utm_x, 
	       y + sin(pose->yaw + dgc_d2r(90)) - origin->utm_y, 
	       z - origin->utm_z);
    glVertex3f(x + cos(pose->yaw - dgc_d2r(90)) - origin->utm_x, 
	       y + sin(pose->yaw - dgc_d2r(90)) - origin->utm_y,
	       z - origin->utm_z);
  }
  glEnd();

  glEndList();
}

void SlamLogfile::DrawPath(void)
{
  glCallList(path_dl);
}

void SlamLogfile::DrawCorrectedPath(void)
{
  glCallList(corrected_path_dl);
}

void SlamInputs::LoadFromFile(char *filename, MatchList *matches,
			      dgc_velodyne_config_p velodyne_config,
			      IntensityCalibration *intensity_calibration)
{
  char log_filename[1000], vlf_filename[1000];
  FILE *fp;
  int optimize;
  int i, j, k, n;
  Match m;

  fp = fopen(filename, "r");
  if (fp == NULL)
    dgc_die("Could not open file %s for reading.\n", filename);
  if(fscanf(fp, "%d\n", &n) != 1)
    dgc_die("Trouble reading from file %s.", filename);
  log_.resize(n);
  for (i = 0; i < n; i++) {
    if(fscanf(fp, "%s %s %d", (char *)&log_filename, (char *)&vlf_filename, 
	   &optimize) != 3)
      dgc_die("Trouble reading from file %s.", filename);
    log(i)->SetLogfileNames(log_filename, vlf_filename);
    log(i)->SetVelodyneConfig(velodyne_config, intensity_calibration);
    log(i)->set_optimize(optimize);
  }

  if(fscanf(fp, "%d", &n) != 1)
    dgc_die("Trouble reading from file %s.", filename);
  for (i = 0; i < n; i++) {
    if(fscanf(fp, "%d %d %d %d", &m.tnum1, &m.snum1, &m.tnum2, &m.snum2) != 4)
      dgc_die("Trouble reading from file %s.", filename);
    for (j = 0; j < 4; j++) {
      for (k = 0; k < 4; k++)
        if(fscanf(fp, "%lf", &(m.offset[j][k])) != 1)
          dgc_die("Trouble reading from file %s.", filename);
    }
    if(fscanf(fp, "%d", &m.optimized) != 1)
      dgc_die("Trouble reading from file %s.", filename);
    matches->AddMatch(m);
  }
  fclose(fp);
}

void SlamInputs::DiscoverInputFiles(int argc, char **argv,
				    dgc_velodyne_config_p velodyne_config,
				    IntensityCalibration *intensity_calibration)
{
  int i;

  log_.resize(argc - 1);
  for (i = 1; i < argc; i++) {
    if (log(i - 1)->DiscoverLogfileNames(argv[i]) < 0)
      dgc_die("Usage: %s [logfile1] ... [logfileN]\n", argv[0]);
    log(i - 1)->SetVelodyneConfig(velodyne_config, intensity_calibration);
  }

  if (log_.size() == 0)
      dgc_die("Requires at least one logfile.\nUsage: %s [logfiles]\n", 
	      argv[0]);

  for (i = 0; i < num_logs(); i++) {
    fprintf(stderr, "%d : %s\n    %s\n", i, log_[i].log_filename(),
	    log_[i].vlf_filename());
  }
}

void SlamInputs::BuildPathKdtrees(void)
{
  int i;

  for (i = 0; i < num_logs(); i++) 
    log(i)->BuildPathKdtree();
}



