#include <roadrunner.h>
#include <velocore.h>
#include <logio.h>
#include <velodyne_interface.h>
#include <applanix_interface.h>
#include "velo_support.h"

namespace dgc {

dgc_velodyne_spin::dgc_velodyne_spin()
{
  scans = (dgc_velodyne_scan_p)malloc(MAX_NUM_VELOSCANS * 
				      sizeof(dgc_velodyne_scan_t));
  dgc_test_alloc(scans);
  num_scans = 0;
}

dgc_velodyne_spin::dgc_velodyne_spin( const dgc_velodyne_spin &copy )
{
  scans = (dgc_velodyne_scan_p)malloc( copy.num_scans * 
      sizeof(dgc_velodyne_scan_t));
  dgc_test_alloc(scans);
  //memcpy( &scans, &(copy.scans), MAX_NUM_VELOSCANS * 
  //    sizeof(dgc_velodyne_scan_t) );
  memcpy( scans, copy.scans, copy.num_scans * 
      sizeof(dgc_velodyne_scan_t) );
  num_scans = copy.num_scans;
}

void dgc_velodyne_spin::copy( const dgc_velodyne_spin &copy )
{
  memcpy( scans, copy.scans, copy.num_scans *
        sizeof(dgc_velodyne_scan_t) );
  num_scans = copy.num_scans;
}

dgc_velodyne_spin::~dgc_velodyne_spin()
{
  free(scans);
}

void dgc_velodyne_spin::save(char *filename, double applanix_lat,
			     double applanix_lon, double applanix_altitude)
{
  FILE *fp;

  fp = fopen(filename, "w");
  if (fp == NULL)
    dgc_die("Error: coukd not open file %s for writing.\n", filename);

  int dummy = fwrite(&num_scans, sizeof(int), 1, fp);
  dummy = fwrite(scans, sizeof(dgc_velodyne_scan_t), num_scans, fp);
  dummy = fwrite(&applanix_lat, sizeof(double), 1, fp);
  dummy = fwrite(&applanix_lon, sizeof(double), 1, fp);
  dummy = fwrite(&applanix_altitude, sizeof(double), 1, fp);
  fclose(fp);
}

int dgc_velodyne_spin::load(char *filename, double *applanix_lat,
			    double *applanix_lon, double *applanix_altitude)
{
  FILE *fp;

  fp = fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n", filename);
    return -1;
  }
  
  int dummy = fread(&num_scans, sizeof(int), 1, fp);
  // [cp] was in Jesse's code: 
  // scans = (dgc_velodyne_scan_p)malloc( num_scans*sizeof(dgc_velodyne_scan_t));
  // dgc_test_alloc(scans);
  dummy = fread(scans, sizeof(dgc_velodyne_scan_t), num_scans, fp);
  dummy = fread(applanix_lat, sizeof(double), 1, fp);
  dummy = fread(applanix_lon, sizeof(double), 1, fp);
  dummy = fread(applanix_altitude, sizeof(double), 1, fp);
  fclose(fp);
  return 0;
}

void dgc_velodyne_spin::load(dgc_velodyne_file_p velodyne,
			     dgc_velodyne_config_p config,
			     dgc_velodyne_index *vindex, int which,
			     double *applanix_lat, double *applanix_lon,
			     double *applanix_alt)
{
  int i, start, err, first = 1, encoder, last_encoder = 0, current_pose = 0;
  dgc_pose_t pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  dgc_velodyne_packet_t pkt;

  num_scans = 0;
  dgc_fseek(velodyne->fp, vindex->spin[which].file_offset, SEEK_SET);
  
  while(1) {
    err = dgc_velodyne_read_packet(velodyne, &pkt);

    if(first) {
      start = vindex->spin[which].spin_start_i;
      first = 0;
    }
    else
      start = 0;
    for(i = start; i < VELO_SCANS_IN_PACKET; i++) {
      encoder = (pkt.scan[i].encoder + config->spin_start) % VELO_NUM_TICKS;

      if(encoder < last_encoder) {    
	*applanix_lat = vindex->spin[which].pose[0].latitude;
	*applanix_lon = vindex->spin[which].pose[0].longitude;
	*applanix_alt = vindex->spin[which].pose[0].altitude;
	return;
      }

      if(current_pose < vindex->spin[which].num_poses && 
	 num_scans == vindex->spin[which].pose[current_pose].scan_num) {
	pose.x = vindex->spin[which].pose[current_pose].smooth_x;
	pose.y = vindex->spin[which].pose[current_pose].smooth_y;
	pose.z = vindex->spin[which].pose[current_pose].smooth_z;
	pose.roll = vindex->spin[which].pose[current_pose].roll;
	pose.pitch = vindex->spin[which].pose[current_pose].pitch;
	pose.yaw = vindex->spin[which].pose[current_pose].yaw;
	current_pose++;
      }
      dgc_velodyne_project_measurement(config, &pkt.scan[i],
				       &scans[num_scans], pose);
      num_scans++;
      last_encoder = encoder;
    }
  }
}

int read_applanix_message(dgc_FILE *log_fp, LineBuffer *line_buffer, 
			  ApplanixPose *applanix_pose)
{
  char *line = NULL, *s = NULL;

  do {
    line = line_buffer->ReadLine(log_fp);
    if(line == NULL) 
      return -1;
    if(strncmp(line, "APPLANIX_POSE_V2", 16) == 0) 
      s = StringV2ToApplanixPose(dgc_next_word(line), applanix_pose);
  } while(s == NULL);
  return 0;
}

int read_applanix_message(dgc_FILE *log_fp, LineBuffer *line_buffer,
    ApplanixPose *applanix_pose, LocalizePose *localize_pose)
{
  char *line = NULL, *s = NULL;

  do {
    line = line_buffer->ReadLine(log_fp);
    if(line == NULL) 
      return -1;
    if(strncmp(line, "APPLANIX_POSE_V2", 16) == 0) 
      s = StringV2ToApplanixPose(dgc_next_word(line), applanix_pose);
    else if (strncmp(line, "LOCALIZE_POSE2", 14) == 0)
      StringV2ToLocalizePose(dgc_next_word(line), localize_pose);
  } while(s == NULL);
  return 0;
}

void vlf_projector(char *velo_log_filename, char *ipc_log_filename,
		   char *cal_filename, char *int_filename, dgc_transform_t velodyne_offset,
		   spin_function f)
{
  ApplanixPose applanix_pose, next_applanix_pose;
  dgc_pose_t pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  int i, err, encoder, last_encoder = 0;
  LineBuffer *line_buffer = NULL;
  dgc_velodyne_config_p config = NULL;
  dgc_velodyne_file_p velodyne = NULL;
  dgc_velodyne_packet_t pkt;
  bool incomplete = false;
  dgc_FILE *log_fp = NULL;
  dgc_velodyne_spin spin;

  /* prepare for velodyne */
  velodyne = dgc_velodyne_open_file(velo_log_filename);
  if(velodyne == NULL)
    dgc_die("Error: could not open velodyne file %s\n", velo_log_filename);
  dgc_velodyne_get_config(&config);
  if(dgc_velodyne_read_calibration(cal_filename, config) != 0) {
    fprintf(stderr, "# ERROR: could not read calibration file!\n");
    exit(0);
  }
  if(dgc_velodyne_read_intensity(int_filename, config) != 0) {
    fprintf(stderr, "# ERROR: could not read intensity calibration file!\n");
    exit(0);
  }
  dgc_velodyne_integrate_offset(velodyne_offset, config);

  /* prepare IPC logfile */
  log_fp = dgc_fopen(ipc_log_filename, "r");
  if(log_fp == NULL)
    dgc_die("Error: could not open file %s for reading.\n", ipc_log_filename);

  line_buffer = new LineBuffer;

  /* read two applanix messages */
  err = read_applanix_message(log_fp, line_buffer, &applanix_pose);
  if(err < 0)
    return;
  err = read_applanix_message(log_fp, line_buffer, &next_applanix_pose);
  if(err < 0)
    return;

  while(1) {
    /* read a velodyne packet */
    err = dgc_velodyne_read_packet(velodyne, &pkt);
    if(err < 0)
      return;

    /* remember if we got a packet before the 1st pose */
    if(pkt.timestamp < applanix_pose.timestamp) 
      incomplete = true;
    /* read applanix messages until the velodyne data falls
       between the timestamps */
    else
      while(next_applanix_pose.timestamp < pkt.timestamp) {
	memcpy(&applanix_pose, &next_applanix_pose, sizeof(ApplanixPose));
	err = read_applanix_message(log_fp, line_buffer, &next_applanix_pose);
	if(err < 0)
	  return;
      }

    /* project the scan according to the current pose */
    for(i = 0; i < VELO_SCANS_IN_PACKET; i++) {
      encoder = (pkt.scan[i].encoder + config->spin_start) % VELO_NUM_TICKS;
      if(encoder < last_encoder) {    
	if(!incomplete)
	  f(&spin, config, &applanix_pose);
	spin.num_scans = 0;
	incomplete = false;
      }
      
      if(spin.num_scans < MAX_NUM_VELOSCANS) {
	pose.x = applanix_pose.smooth_x;
	pose.y = applanix_pose.smooth_y;
	pose.z = applanix_pose.smooth_z;
	pose.roll = applanix_pose.roll;
	pose.pitch = applanix_pose.pitch;
	pose.yaw = applanix_pose.yaw;
	dgc_velodyne_project_measurement(config, &(pkt.scan[i]), 
					 &(spin.scans[spin.num_scans]), pose);
	spin.scans[spin.num_scans].timestamp = pkt.timestamp; 
	spin.num_scans++;
      }
      last_encoder = encoder;
    }
  }
}

dgc_velodyne_index::dgc_velodyne_index()
{
  num_spins = 0;
  spin = NULL;
}

dgc_velodyne_index::~dgc_velodyne_index()
{
  int i;

  for(i = 0; i < num_spins; i++)
    free(spin[i].pose);
  free(spin);
}

int dgc_velodyne_index::load(char *filename)
{
  int n, max_spins = 0;
  off64_t offset;
  dgc_FILE *fp;

  fp = dgc_fopen(filename, "r");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for reading.\n", filename);
    return -1;
  }
  
  do {
    n = dgc_fread(&offset, sizeof(off64_t), 1, fp);
    if(n == 1) {
      if(num_spins == max_spins) {
	max_spins += 10000;
	spin = (dgc_velodyne_index_entry *)
	  realloc(spin, max_spins * sizeof(dgc_velodyne_index_entry));
	dgc_test_alloc(spin);
      }

      spin[num_spins].file_offset = offset;
      dgc_fread(&spin[num_spins].spin_start_i, sizeof(int), 1, fp);
      dgc_fread(&spin[num_spins].num_scans, sizeof(int), 1, fp);
      dgc_fread(&spin[num_spins].num_poses, sizeof(int), 1, fp);

      spin[num_spins].pose = 
	(dgc_velodyne_index_pose *)calloc(spin[num_spins].num_poses,
					  sizeof(dgc_velodyne_index_pose));
      dgc_test_alloc(spin[num_spins].pose);

      dgc_fread(spin[num_spins].pose, sizeof(dgc_velodyne_index_pose), 
	       spin[num_spins].num_poses, fp);

      num_spins++;
    }
  } while(n == 1);
  dgc_fclose(fp);
  return 0;
}

int dgc_velodyne_index::save(char *filename)
{
  int i, percent, last_percent = -1;
  dgc_FILE *fp;

  fp = dgc_fopen(filename, "w");
  if(fp == NULL) {
    fprintf(stderr, "Error: could not open file %s for writing.\n", filename);
    return -1;
  }
  
  for(i = 0; i < num_spins; i++) {
    dgc_fwrite(&spin[i].file_offset, sizeof(off64_t), 1, fp);
    dgc_fwrite(&spin[i].spin_start_i, sizeof(int), 1, fp);
    dgc_fwrite(&spin[i].num_scans, sizeof(int), 1, fp);
    dgc_fwrite(&spin[i].num_poses, sizeof(int), 1, fp);
    dgc_fwrite(spin[i].pose, sizeof(dgc_velodyne_index_pose), 
	      spin[i].num_poses, fp);
    percent = (int)rint(i / (double)num_spins * 100.0);
    if(percent != last_percent) {
      fprintf(stderr, "\rSaving %s (%d%%)    ", filename, percent);
      last_percent = percent;
    }
  }
  fprintf(stderr, "\rSaving %s (100%%)    \n", filename);
  dgc_fclose(fp);
  return 0;
}

}
