#ifndef DGC_IBEOCORE_H
#define DGC_IBEOCORE_H

#ifdef __cplusplus
extern "C" {
#endif

#define    IBEO_BUFFER_SIZE        1000000
#define    IBEO_READ_TIMEOUT       0.25
#define    SCAN_QUEUE_LENGTH       20 
#define    IBEO_SCAN_PACKET_ID     15

typedef struct {
  unsigned char scanner_id;
  unsigned char level_num, secondary, status;
  float x, y, z;
} dgc_ibeo_point_t, *dgc_ibeo_point_p;

typedef struct {
  double timestamp, ibeo_timestamp;
  float start_angle, end_angle;
  int scan_counter;
  int num_points;
  dgc_ibeo_point_t point[8648];
} dgc_ibeo_scan_t, *dgc_ibeo_scan_p;

typedef struct {
  int sock;
  int laser_num;

  int buffer_position, processed_mark;
  unsigned char buffer[IBEO_BUFFER_SIZE];

  int num_scans;
  dgc_ibeo_scan_t scan[SCAN_QUEUE_LENGTH];

  double latest_timestamp;
} dgc_ibeo_t, *dgc_ibeo_p;

dgc_ibeo_p dgc_ibeo_connect(char *host, int port);

void dgc_ibeo_process(dgc_ibeo_p ibeo);

void dgc_ibeo_disconnect(dgc_ibeo_p *ibeo);

#ifdef __cplusplus
}
#endif

#endif
