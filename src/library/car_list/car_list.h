/*
 *  Created on: Nov 24, 2009
 *      Author: duhadway
 */

#ifndef CAR_LIST_H_
#define CAR_LIST_H_

#include <vector>
#include <obstacle_types.h>

struct car_pose_t {
  double x, y, theta;
  double timestamp;
  int t;
  TurnSignalState signal;
};

class car_t {
public:
  std::vector <car_pose_t> pose;
  double w, l;
  int fixed, start_cap, end_cap;

  car_pose_t *get_pose(int t);
  bool estimate_pose(int t, double *x, double *y, double *theta,
      bool *extrapolated);
  bool estimate_pose(double t, double *x, double *y, double *theta,
      bool *extrapolated);
  bool estimate_signal(int t, TurnSignalState* signal);
  bool estimate_signal(double t, TurnSignalState* signal);
  bool estimate_velocity(int t, double *vel, double dt);
  bool estimate_velocity(double t, double *vel);
  void delete_pose(int t);
  bool center_selected(double x, double y, int t);
  bool front_selected(double x, double y, int t);
  bool side_selected(double x, double y, int t);
  bool car_selected(double x, double y, int t);
  bool corner_selected(double x, double y, int t);
  int num_poses(void) { return (int)pose.size(); }
private:
};

class car_list_t {
public:
  std::vector <car_t> car;
  int num_cars(void) { return (int)car.size(); }
  void add_car(double x, double y, double theta, double w, double l, int t, double time);
  void rotate_car(float dtheta);
  void save_labels(char *filename);
  void save_labels_simple(char *filename, int num_spins);
  void load_labels(char *filename);
};

#endif /* CAR_LIST_H_ */
