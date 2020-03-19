#ifndef DGC_LDLRSCORE_H
#define DGC_LDLRSCORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <roadrunner.h>

#define    LDLRS_BUFFER_SIZE        1000000
#define    LDLRS_READ_TIMEOUT       0.25
#define    SCAN_QUEUE_LENGTH        20 

typedef struct {
  float range, intensity;
} dgc_ldlrs_point_t;

typedef struct {
  int profiles_sent, profile_count;
  double start_angle, end_angle, angle_step;
  int sector_start_ts, sector_end_ts;
  int num_points;
  float range[360 * 8];
  short int intensity[360 * 8];
  double timestamp;
} dgc_ldlrs_scan_t, *dgc_ldlrs_scan_p;

typedef struct {
  int sock;
  int laser_num;
  
  int buffer_position, processed_mark;
  unsigned char buffer[LDLRS_BUFFER_SIZE];

  int num_scans;
  dgc_ldlrs_scan_t scan[SCAN_QUEUE_LENGTH];

  double latest_timestamp;

  char serialnum[20];

} dgc_ldlrs_t, *dgc_ldlrs_p;

dgc_ldlrs_p dgc_ldlrs_connect(char *host, int port, int motor_speed,
			      double start_angle, double end_angle,
			      double resolution);

void dgc_ldlrs_disconnect(dgc_ldlrs_p *ldlrs);

void dgc_ldlrs_process(dgc_ldlrs_p ldlrs);

#ifdef __cplusplus
}
#endif

#endif
