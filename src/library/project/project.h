#ifndef DGC_PROJECT_H
#define DGC_PROJECT_H

#include <deque>

class timed_data {
public:
  virtual ~timed_data(void) {};
  double timestamp;
};

class pose_data : public timed_data {
public:
  double x, y, z, roll, pitch, yaw;
};

class reference_queue {
 public:
  reference_queue(int ref_queue_size);
  ~reference_queue(void);
  void push(timed_data *d);
 private:
  std::deque <timed_data *> reference;
  int ref_queue_size;
  friend class sensor_queue;
};

class sensor_queue {
 public:
  sensor_queue(int sensor_queue_size);
  ~sensor_queue(void);
  void push(timed_data *d);
  int bracket_data(reference_queue *q, timed_data **d, 
                   timed_data **before, timed_data **after);
  int posestamped_data(reference_queue *q, timed_data **d, pose_data *pose);
 private:
  std::deque <timed_data *> sensor;
  int sensor_queue_size;
};

#endif
