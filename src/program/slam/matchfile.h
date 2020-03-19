#ifndef DGC_MATCHFILE_H
#define DGC_MATCHFILE_H

#include <roadrunner.h>
#include <transform.h>

typedef struct {
  int traj, spin;
} spin_id_t, *spin_id_p;

typedef struct {
  spin_id_t pose1, pose2;
  int num_extras;
  spin_id_t extra[10];
  dgc_transform_t t;
  double offset_x, offset_y, offset_z;
} match_t, *match_p;

class matchfile_t {
public:
  match_p match;
  int num_matches, max_matches;

  matchfile_t();
  ~matchfile_t();
  void add_match(match_p m);
  void remove_unused_poses(void);
  void sort_matches(void);
  void save(char *filename);
  void load(char *filename);
};

#endif
