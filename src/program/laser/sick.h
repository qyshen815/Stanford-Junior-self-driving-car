#ifndef DGC_SICK_H
#define DGC_SICK_H

namespace dgc {

#define CRC16_GEN_POL                    0x8005
#define CRC16_GEN_POL0                   0x80
#define CRC16_GEN_POL1                   0x05

#define SICK_WRITE_TIMEOUT               0.1
#define SICK_ACK_TIMEOUT                 0.1

#define SICK_LMSTYPE_TIMEOUT             0.5
#define SICK_LMSSTATUS_TIMEOUT           1.0
#define SICK_SET_CONFIG_TIMEOUT          3.0
#define SICK_GET_CONF_TIMEOUT            0.3
#define SICK_SET_CONF_TIMEOUT            3.0
#define SICK_SET_RES_TIMEOUT             1.0
#define SICK_CONTINUOUS_TIMEOUT          1.0
#define SICK_READ_TIMEOUT                0.25

#define ACK                              0x06

#define STANDARD_SENSITIVITY             0x00
#define MEDIUM_SENSITIVITY               0x01
#define LOW_SENSITIVITY                  0x02
#define HIGH_SENSITIVITY                 0x03

#define RETRY_COUNT                      5

#define SCAN_QUEUE_LENGTH                20

#define TIMESTAMP_OBSERVATION_VARIANCE   (0.05*0.05)

#define LASER_BUFFER_SIZE                100000

typedef struct {
  char device[256];
  int detect_baudrate;
  int set_baudrate;
  int laser_num;
  int angular_range;
  float angular_resolution;
  int read_intensity;
} sick_laser_param_t, *sick_laser_param_p;

typedef struct {
  int num_range_readings;
  float range[1000];
  int num_intensity_readings;
  unsigned char intensity[1000];
  double timestamp;
} dgc_laser_scan_t, *dgc_laser_scan_p;

typedef struct {
  sick_laser_param_t param;
  int fd;
  char software_version[20], product_name[20], product_type[20];
  float fov;

  char serialnum[10];

  pthread_mutex_t mutex;

  unsigned char buffer[LASER_BUFFER_SIZE];
  int buffer_position, processed_mark;
  long int packet_offset, packet_length;
  int first_timestamp;
  double timestamp_mean, timestamp_variance;
  double latest_timestamp;
  int num_scans;
  dgc_laser_scan_t scan_queue[SCAN_QUEUE_LENGTH];
} sick_laser_t, *sick_laser_p;

sick_laser_p sick_laser_start(sick_laser_param_p param);

void sick_sleep_until_input(sick_laser_p laser, double timeout);

void sick_process_laser(sick_laser_p laser);

void sick_laser_stop(sick_laser_p laser);

}

#endif
