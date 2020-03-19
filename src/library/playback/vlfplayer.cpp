#include <roadrunner.h>
#include <velodyne_interface.h>
#include "vlfplayer.h"

namespace dgc {

#define       MAX_NUM_SCANS                    20000
#define       VELODYNE_BYTES_PER_SECOND        3100000.0

vlf_player::vlf_player(VelodyneInterface *vint)
{
  num_scans = 0;
  scans = NULL;
  config = NULL;
  velodyne = NULL;  
  last_encoder = 0;
  velo_interface = vint;
}

vlf_player::~vlf_player()
{
  if(scans != NULL)
    free(scans);
}

int vlf_find_start(dgc_FILE *fp, double *timestamp, off64_t *pos)
{
  unsigned char data[16];
  unsigned short len;
  int n;

  while(1) {
    do {
      *pos = dgc_ftell(fp);
      n = dgc_fgetc(fp);
      if(n == EOF) 
	return -1;
    } while(n != VLF_START_BYTE);
    
    n = dgc_fread(data, 8, 1, fp);
    if(n != 1) 
      return -1;
    memcpy(timestamp, data, 8);

    n = dgc_fread(data, 2, 1, fp);
    if(n != 1) 
      return -1;
    memcpy(&len, data, 2);
    
    if(len == VELO_PACKET_SIZE) {
      dgc_fseek(fp, *pos, SEEK_SET);
      return 0;
    }
  }
}

int vlf_player::initialize(char *vlf_filename, char *cal_filename_param, char *int_filename_param,
			   dgc_transform_t velodyne_offset_param)
{
  /* allocate scans */
  scans = 
    (dgc_velodyne_scan_p)malloc(MAX_NUM_SCANS * sizeof(dgc_velodyne_scan_t));
  dgc_test_alloc(scans);

  /* read logfile */
  velodyne = dgc_velodyne_open_file(vlf_filename);
  if(velodyne == NULL) {
    fprintf(stderr, "Error: could not open velodyne file %s\n", vlf_filename);
    return -1;
  }

  /* read calibration */
  dgc_velodyne_get_config(&config);
  if(dgc_velodyne_read_calibration(cal_filename_param, config) != 0) {
    fprintf(stderr, "Error: could not read calibration file!\n");
    return -1;
  }
  if (dgc_velodyne_read_intensity(int_filename_param, config ) != 0) {
    fprintf(stderr, "Error: could not read intensity calibration file!\n");
    return -1;
  }

  dgc_velodyne_integrate_offset(velodyne_offset_param, config);
  
  /* find first valid file offset and timestamp */
  if(vlf_find_start(velodyne->fp, &first_packet_ts, &last_packet_fpos) == -1) {
    fprintf(stderr, "Could not find valid packet in VLF file.\n");
    return -1;
  }
  return 0;
}

void vlf_player::seek(double t)
{
  off64_t new_pos;
  double next_ts;

  t -= first_packet_ts;

  if(t < 0)
    new_pos = 0;
  else
    new_pos = (off64_t)(t * VELODYNE_BYTES_PER_SECOND);

  if(dgc_fseek(velodyne->fp, new_pos, SEEK_SET) != 0) 
    dgc_die("# ERROR: skipping %.1f seonds - fseek failed.\n", t);
  vlf_find_start(velodyne->fp, &next_ts, &new_pos);

  set_last_packet_time(next_ts);
  num_scans = 0;
}

void vlf_player::read_packet(double t, dgc_pose_p pose, double max_age)
{
  dgc_velodyne_packet_t pkt;
  int encoder, i, err;
  off64_t fpos;

  /* read a new packet */
  err = dgc_velodyne_read_packet(velodyne, &pkt);
  if(err < 0) {
    set_eof(true);
    return;
  }

  set_last_packet_time(pkt.timestamp);
  fpos = dgc_ftell(velodyne->fp);
  add_input_bytes(fpos - last_packet_fpos);
  last_packet_fpos = fpos;
   
  /* project the beams, and publish over shared memory */
  for(i = 0; i < VELO_SCANS_IN_PACKET; i++) {
    encoder = (pkt.scan[i].encoder + 18000) % 36000;
    if(encoder < last_encoder) {	
      if(num_scans > 0 &&
	 fabs(scans[num_scans - 1].timestamp - t) < max_age) {
	velo_interface->WriteScans(num_scans, scans);
	add_output_bytes(num_scans * sizeof(dgc_velodyne_scan_t));
	add_frames(1);
      }
      num_scans = 0;
    }
    if(num_scans < MAX_NUM_SCANS) {
      dgc_velodyne_project_measurement(config, &(pkt.scan[i]), 
				       &(scans[num_scans]), *pose);
      scans[num_scans].timestamp = pkt.timestamp;
      num_scans++;
    }
    last_encoder = encoder;
  }
}

}
