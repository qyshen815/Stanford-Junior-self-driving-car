#ifndef DGC_RIEGLCORE_H
#define DGC_RIEGLCORE_H

#include <ipc_interface.h>

namespace dgc {

class RieglServer {
 public:
  RieglServer(IpcInterface *ipc);

  int Connect(char *host, int laser_num, double start_angle, 
	      double angular_resolution, int num_readings, int get_intensity, 
	      int get_angle, int get_quality, int get_sync);
  int ReadScan(void);
  void Disconnect(void);

  void RegisterIpcMessages(void);
  void PublishLaser(void);

 private:

  int SendCommand(char *cmd, int command_length);
  int ReadResponse(char *response, int *response_length, double read_timeout);
  int CommandResponse(char *cmd, char *description, double timeout);
  int GetParamVal(char *cmd, char *description, char *response);
  int StartProgrammingMode(void);
  int QuitProgrammingMode(void);
  int SetStartAngle(double start_angle);
  int SetAngularResolution(double resolution);
  int SetNumReadings(int num_readings);
  int StartContinuousMode(void);
  int SetPayload(int get_range, int get_intensity, int get_angle,
		 int get_quality, int get_sync);
  int ApplySettings(void);
  int GetApplyError(void);
  int ReadDataHeader(void);

  IpcInterface *ipc_;

  int control_fd;
  int data_fd;

  char serialnum[10];

  float start_angle;
  float fov;

  int param_change;

  unsigned int dataset_len;
  int sync_present;
  int crc_present;
  unsigned int data_offset;
  unsigned int datapoint_len;
  unsigned int num_datapoints;

  unsigned char mirror_sides;
  
  int has_range;
  float *range;

  int has_intensity;
  unsigned char *intensity;

  int has_angle;
  float *angle;

  int has_quality;
  unsigned char *quality;

  int has_sync;
  float *shot_timestamp;

  float range_unit;
  float angle_unit;
  float timer_unit;

  int line_counter;
  int sync_counter;
  double line_timestamp;
  unsigned char *read_buffer;

  int laser_num;
};

}

#endif
