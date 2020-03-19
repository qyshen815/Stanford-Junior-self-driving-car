#include <roadrunner.h>
#include <data_player.h>

#define    DATA_MIN_SKIP            1.0
#define    DATA_MAX_AGE             0.2

data_player::data_player()
{
  i_bytes = 0;
  o_bytes = 0;
  frame_count = 0;

  pthread_mutex_init(&input_mutex, NULL);
  pthread_cond_init(&input_cond, NULL);

  last_packet_time = 0;

  /* timing variables */
  i_speed = 0;
  o_speed = 0;
  hz = 0.0; 
  first = 1;
}

data_player::~data_player()
{
  pthread_mutex_destroy(&input_mutex);
  pthread_cond_destroy(&input_cond);
}

void data_player::reset(void)
{
  pthread_mutex_lock(&input_mutex);
  reset_playback = true;
  pthread_cond_signal(&input_cond);
  pthread_mutex_unlock(&input_mutex);
}

void data_player::disable_playback(void)
{
  pthread_mutex_lock(&input_mutex);
  stop_playback = true;
  pthread_cond_signal(&input_cond);
  pthread_mutex_unlock(&input_mutex);
  pthread_join(thread, NULL);
}

static void *player_thread(void *ptr)
{
  data_player *player = (data_player *)ptr;

  player->play();
  return NULL;
}

void data_player::enable_playback(void)
{
  pthread_create(&thread, NULL, player_thread, this);
}

void data_player::play(void)
{
  double current_time_copy;
  dgc_pose_t pose_copy;

  stop_playback = false;
  reset_playback = true;
  while(!stop_playback) {
    /* wait for some input */
    pthread_mutex_lock(&input_mutex);
    pthread_cond_wait(&input_cond, &input_mutex);

    /* if we get reset signal, go back to first packet */
    if(reset_playback) {
      seek(0);
      current_time = 0;
      found_eof = false;
      reset_playback = false;
    }
    
    /* copy the input variables */
    current_time_copy = current_time;
    pose_copy = pose;
    pthread_mutex_unlock(&input_mutex);
    
    /* skip to right place */ 
    if(fabs(current_time_copy - last_packet_time) > DATA_MIN_SKIP && 
       !found_eof) 
      seek(dgc_fmax(current_time_copy - 0.001, 0));
    
    while(last_packet_time < current_time_copy && !found_eof) 
      read_packet(current_time_copy, &pose_copy, DATA_MAX_AGE);
  }
}

void data_player::set_eof(bool eof)
{
  found_eof = eof;
}

void data_player::set_last_packet_time(double t)
{
  last_packet_time = t;
}

void data_player::set_pose(double x, double y, double z, 
			  double roll, double pitch, double yaw,
			  double timestamp)
{
  pthread_mutex_lock(&input_mutex);
  if(!found_eof) {
    pose.x = x;
    pose.y = y;
    pose.z = z;
    pose.roll = roll;
    pose.pitch = pitch;
    pose.yaw = yaw;
    current_time = timestamp;
  }
  pthread_cond_signal(&input_cond);
  pthread_mutex_unlock(&input_mutex);
}

void data_player::throughput_stats(double *disk_rate, double *shm_rate,
				   double *freq)
{
  double t, dt;

  t = last_packet_time;
  if(first) {
    i_bytes = 0;
    o_bytes = 0;
    frame_count = 0;
    last_stat_check = t;
    first = 0;
    dt = 0;
  }
  else
    dt = t - last_stat_check;

  if(dt > 1.0) {
    i_speed = i_bytes / dt / (1024.0 * 1024.0);
    o_speed = o_bytes / dt / (1024.0 * 1024.0);
    hz = frame_count / dt;
    i_bytes = 0;
    o_bytes = 0;
    frame_count = 0;
    last_stat_check = t;
  }

  *disk_rate = i_speed;
  *shm_rate = o_speed;
  *freq = hz;
}

void data_player::add_input_bytes(int input_bytes)
{
  i_bytes += input_bytes;
}

void data_player::add_output_bytes(int output_bytes)
{
  o_bytes += output_bytes;
}

void data_player::add_frames(int frames)
{
  frame_count += frames;
}
