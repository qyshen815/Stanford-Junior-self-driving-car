#include <roadrunner.h>
#include <kdtree.h>
#include <logio.h>
#include <lltransform.h>
#include <transform.h>
#include <velo_support.h>
#include "slam_inputs.h"
#include "overlap.h"
#include <vector>

using namespace dgc;
using std::vector;

#define       MAX_MATCH_DIST          5.0
#define       MAX_PREV_MATCH_DIST     30.0

//#define       MAX_MATCH_DIST          10.0
//#define       MAX_PREV_MATCH_DIST     20.0

#define       SEQUENCE_STEP           2
#define       SEQUENCE_NUM_STEPS      2

struct OverlapPose {
  int spin_num;
  double utm_x, utm_y, utm_z, d, smooth_x, smooth_y, smooth_z, timestamp;
  char utmzone[5];
};

class OverlapTrajectory {
public:
  OverlapTrajectory();
  ~OverlapTrajectory();
  void ExtractPoses(dgc_velodyne_index *velodyne_index);
  void BuildKdtree(void);
  
  
  int num_poses(void) { return (int)pose_.size(); }
  OverlapPose &pose(int i) { return pose_[i]; }
private:
  vector <OverlapPose> pose_;

public:
  dgc_kdtree_p pose_kdtree_;
  vector <int> taken_;
};

OverlapTrajectory::OverlapTrajectory(void)
{
  pose_kdtree_ = NULL;
}

OverlapTrajectory::~OverlapTrajectory()
{
  if (pose_kdtree_)
    dgc_kdtree_free(&pose_kdtree_);
}

void OverlapTrajectory::ExtractPoses(dgc_velodyne_index *index)
{
  double last_x = 0, last_y = 0, d, total_d = 0;
  OverlapPose p;
  int i;

  for(i = 0; i < index->num_spins; i++) {
    d = hypot(index->spin[i].pose[0].smooth_x - last_x,
	      index->spin[i].pose[0].smooth_y - last_y);
    if(i == 0 || d > 0.2) { 
      last_x = index->spin[i].pose[0].smooth_x;
      last_y = index->spin[i].pose[0].smooth_y;
      if(i != 0)
	total_d += d;

      p.smooth_x = index->spin[i].pose[0].smooth_x;
      p.smooth_y = index->spin[i].pose[0].smooth_y;
      p.smooth_z = index->spin[i].pose[0].smooth_z;
      
      ConvertToUtm(index->spin[i].pose[0].latitude,
		   index->spin[i].pose[0].longitude,
		   index->spin[i].pose[0].altitude,
		   &p.utm_y, &p.utm_x, &p.utm_z, p.utmzone);
      p.d = total_d;
      p.timestamp = index->spin[i].pose[0].timestamp;
      p.spin_num = i;
      pose_.push_back(p);
    }
  }
}

void OverlapTrajectory::BuildKdtree(void)
{
  double *x, *y;
  int i, n = num_poses();

  x = new double[n];
  y = new double[n];
  for(i = 0; i < n; i++) {
    x[i] = pose(i).utm_x;
    y[i] = pose(i).utm_y;
  }
  pose_kdtree_ = dgc_kdtree_build_balanced(x, y, n);
  delete [] x;
  delete [] y;
}

void FindOverlap(SlamInputs *inputs, MatchList *matches)
{
  vector <OverlapTrajectory *> traj;
  OverlapTrajectory *t1, *t2;
  int i, j, min_j, t1_num, t2_num, last_i;
  dgc_dlist_node_p dlist, temp;
  int added_match, bad_pose, best_pose;
  double d, best_d, min_d;
  Match m;
  
  matches->Clear();

  traj.resize(inputs->num_logs());
  for (i = 0; i < inputs->num_logs(); i++) {
    traj[i] = new OverlapTrajectory;
    traj[i]->ExtractPoses(inputs->log(i)->index());
    traj[i]->BuildKdtree();
  }
  
  for(t1_num = 0; t1_num < inputs->num_logs(); t1_num++) {
    t1 = traj[t1_num];

    for(t2_num = t1_num; t2_num < inputs->num_logs(); t2_num++) {
      t2 = traj[t2_num];

      /* don't add matches between two fixed trajectories */
      if(!inputs->log(t1_num)->optimize() &&
	 !inputs->log(t2_num)->optimize())
	continue;

      last_i = -1;
      for(i = 0; i < t1->num_poses(); i++) {
	if((last_i == -1 || 
	    t1->pose(i).d - t1->pose(last_i).d > MAX_PREV_MATCH_DIST)) {
	  /* find all neighbors within MATCH_DIST */
	  dlist = dgc_kdtree_range_search(t2->pose_kdtree_, 
					  t1->pose(i).utm_x, 
					  t1->pose(i).utm_y, 
					  MAX_MATCH_DIST);
	  
	  /* loop until you can't find any legal matches from this position */
	  added_match = 0;
	  do {         	               /* find the closest legal match left */
	    temp = dlist;
	    best_pose = -1;
	    best_d = 0;
	    while(temp != NULL) {
	      bad_pose = 0;
	      /* if matching trajectory against itself, make sure a loop has 
		 occured */
	      if(t1_num == t2_num) {
		if(temp->id <= i)
		  bad_pose = 1;
		if(fabs(t1->pose(temp->id).d - 
			t1->pose(i).d) < MAX_MATCH_DIST * 1.5)
		  bad_pose = 1;
	      }
	      

	      if(!bad_pose)
		for(j = 0; j < (int)t2->taken_.size(); j++)
		  if(fabs(t2->pose(temp->id).d - t2->pose(t2->taken_[j]).d) < 
		     MAX_PREV_MATCH_DIST) {
		    bad_pose = 1;
		    break;
		  }
	      
	      if(!bad_pose) {
		d = 
		  hypot(t1->pose(i).utm_x - t2->pose(temp->id).utm_x,
			t1->pose(i).utm_y - t2->pose(temp->id).utm_y);
		if(best_pose == -1 || d < best_d) {
		  best_pose = temp->id;
		  best_d = d;
		}
	      }
	      
	      temp = temp->next;
	    }
	    
	    /* tack the match onto our list */
	    if(best_pose != -1) {
	      t2->taken_.push_back(best_pose);
	      added_match = 1;
	      
	      m.tnum1 = t1_num;
	      m.snum1 = i;
	      m.tnum2 = t2_num;
	      m.snum2 = best_pose;
	      matches->AddMatch(m);
	    }
	  } while(best_pose != -1);

	  dgc_dlist_free(&dlist);
	
	  if(added_match)
	    last_i = i;
	}
      }
    }
  }

  for(i = 0; i < matches->num_matches(); i++) {
    m = *(matches->match(i));

    min_j = m.snum2;
    min_d = hypot(traj[m.tnum2]->pose(min_j).utm_x - 
		  traj[m.tnum1]->pose(m.snum1).utm_x,
		  traj[m.tnum2]->pose(min_j).utm_y - 
		  traj[m.tnum1]->pose(m.snum1).utm_y);
    j = m.snum2 - 1;
    while (j >= 0) {
      d = hypot(traj[m.tnum2]->pose(j).utm_x - 
		traj[m.tnum1]->pose(m.snum1).utm_x,
		traj[m.tnum2]->pose(j).utm_y - 
		traj[m.tnum1]->pose(m.snum1).utm_y);
      if (d <= min_d) {
	min_j = j;
	min_d = d;
	j--;
      } else {
	break;
      }
    }
    j = m.snum2 + 1;
    while (j < traj[m.tnum2]->num_poses() - 1) {
      d = hypot(traj[m.tnum2]->pose(j).utm_x - 
		traj[m.tnum1]->pose(m.snum1).utm_x,
		traj[m.tnum2]->pose(j).utm_y - 
		traj[m.tnum1]->pose(m.snum1).utm_y);
      if (d <= min_d) {
	min_j = j;
	min_d = d;
	j++;
      } else {
	break;
      }
    }
    m.snum2 = min_j;

    matches->match(i)->snum1 = traj[m.tnum1]->pose(m.snum1).spin_num;
    matches->match(i)->snum2 = traj[m.tnum2]->pose(m.snum2).spin_num;
    dgc_transform_identity(matches->match(i)->offset);
    matches->match(i)->optimized = false;
  }

  for (i = 0; i < inputs->num_logs(); i++) 
    delete traj[i];
}

