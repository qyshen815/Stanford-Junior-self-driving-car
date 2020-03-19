#include <roadrunner.h>
#include <project.h>

using std::deque;

double interpolate_yaw(double head1, double head2, double fraction)
{
  double result;

  if(head1 > 0 && head2 < 0 && head1 - head2 > M_PI) {
    head2 += 2 * M_PI;
    result = head1 + fraction * (head2 - head1);
    if(result > M_PI)
      result -= 2 * M_PI;
    return result;
  }
  else if(head1 < 0 && head2 > 0 && head2 - head1 > M_PI) {
    head1 += 2 * M_PI;
    result = head1 + fraction * (head2 - head1);
    if(result > M_PI)
      result -= 2 * M_PI;
    return result;
  }
  else
    return head1 + fraction * (head2 - head1);
}

reference_queue::reference_queue(int ref_queue_size)
{
  this->ref_queue_size = ref_queue_size;
}

reference_queue::~reference_queue(void)
{
  timed_data *d2;

  while(reference.size() > 0) {
    d2 = reference.front();
    reference.pop_front();
    delete d2;
  }
}

void reference_queue::push(timed_data *d)
{
  timed_data *d2;
  deque <timed_data *>::reverse_iterator iter;

  /* walk backwards until we find a place for the new data
     or walk off the end */
  iter = reference.rbegin();
  while(iter != reference.rend() &&
        d->timestamp < (*iter)->timestamp) 
    iter++;
  reference.insert(iter.base(), d);

  while((int)reference.size() > ref_queue_size) {
    d2 = reference.front();
    reference.pop_front();
    delete d2;
  }
}

sensor_queue::sensor_queue(int sensor_queue_size)
{
  this->sensor_queue_size = sensor_queue_size;
}

sensor_queue::~sensor_queue(void)
{
  timed_data *d2;

  while(sensor.size() > 0) {
    d2 = sensor.front();
    sensor.pop_front();
    delete d2;
  }
}

void sensor_queue::push(timed_data *d)
{
  timed_data *d2;

  sensor.push_back(d);
  while((int)sensor.size() > sensor_queue_size) {
    d2 = sensor.front();
    sensor.pop_front();
    delete d2;
  }
}

int sensor_queue::bracket_data(reference_queue *q, timed_data **d, 
                               timed_data **before, timed_data **after)
{
  timed_data *temp;
  deque <timed_data *>::iterator i;

  if(q->reference.size() <= 1)
    return -1;

  /* remove sensor data that comes before all reference data */
  while(sensor.size() > 0 && 
        sensor.front()->timestamp < q->reference.front()->timestamp) {
    temp = sensor.front();
    sensor.pop_front();
    delete temp;
  }

  if(sensor.size() == 0 || sensor.front()->timestamp > 
     q->reference.back()->timestamp) 
    return -1;

  /* find pair of reference data that bracket sensor data */
  i = q->reference.begin();
  while((*i)->timestamp < sensor.front()->timestamp)
    i++;
  i--;
  
  *d = sensor.front();
  sensor.pop_front();
  *before = *i;
  *after = *(i + 1);
  return 0;
}

int sensor_queue::posestamped_data(reference_queue *q, timed_data **d,
                                   pose_data *pose)
{
  timed_data *before, *after;
  pose_data *before_pose, *after_pose;
  int err = bracket_data(q, d, &before, &after);
  double frac;

  if(err == -1)
    return -1;

  before_pose = dynamic_cast<pose_data *>(before);
  after_pose = dynamic_cast<pose_data *>(after);
  if(before_pose == 0 || after_pose == 0)
    dgc_die("Error: posestamp_data method can only be called when the"
            " reference data queue is filled with pose_data objects.\n");

  frac = ((*d)->timestamp - before->timestamp) /
    (after->timestamp - before->timestamp);

  pose->x = before_pose->x + frac * (after_pose->x - before_pose->x);
  pose->y = before_pose->y + frac * (after_pose->y - before_pose->y);
  pose->z = before_pose->z + frac * (after_pose->z - before_pose->z);
  pose->roll = before_pose->roll + frac * (after_pose->roll - 
                                           before_pose->roll);
  pose->pitch = before_pose->pitch + frac * (after_pose->pitch - 
                                             before_pose->pitch);
  pose->yaw = interpolate_yaw(before_pose->yaw, after_pose->yaw, frac);
  return 0;
}
