#ifndef DGC_DATA_PLAYER_H
#define DGC_DATA_PLAYER_H

#include <roadrunner.h>

class data_player {
public:
  data_player();
  virtual ~data_player();

  void play(void);
  void reset(void);

  void enable_playback(void);
  void disable_playback(void);

  void set_pose(double x, double y, double z, double roll,
		double pitch, double yaw, double timestamp);

  void throughput_stats(double *disk_rate, double *shm_rate, double *freq);

 protected:
  void set_eof(bool eof);
  void set_last_packet_time(double t);
  void add_input_bytes(int input_bytes);
  void add_output_bytes(int output_bytes);
  void add_frames(int frames);
  virtual void seek(double t) = 0;
  virtual void read_packet(double t, dgc_pose_p pose, double max_age) = 0;

 private:
  pthread_t thread;
  pthread_mutex_t input_mutex;
  pthread_cond_t input_cond;

  bool found_eof, reset_playback, stop_playback;
  dgc_pose_t pose;
  double current_time, last_packet_time;

  int i_bytes;
  int o_bytes;
  int frame_count;

  double last_stat_check;
  double i_speed;
  double o_speed;
  double hz;
  int first;
};

#endif
