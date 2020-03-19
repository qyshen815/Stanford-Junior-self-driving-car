#include <roadrunner.h>
#include <velocore.h>
#include <velo_support.h>
#include "correctindex.h"
#include "slam.h"
#include <vector>

using namespace dgc;
using std::vector;

#define     STD_ODOM      0.01
#define     STD_MATCH     0.01
#define     STD_GPS       1

void ExtractPoses(SlamInputs *inputs, vector <vector <SlamPose> > &t)
{
  dgc_velodyne_index_pose *pose;
  dgc_velodyne_index *index;
  char utmzone[10];
  SlamPose p;
  int i, j;

  t.resize(inputs->num_logs());
  for (i = 0; i < (int)t.size(); i++) {
    index = inputs->log(i)->index();
    t[i].clear();
    for (j = 0; j < index->num_spins; j++) {
      pose = &(index->spin[j].pose[0]);
      p.smooth_x = pose->smooth_x;
      p.smooth_y = pose->smooth_y;
      p.smooth_z = pose->smooth_z;
      ConvertToUtm(pose->latitude, pose->longitude, pose->altitude,
		   &p.utm_x, &p.utm_y, &p.utm_z, utmzone);
      p.fixed_x = 0;
      p.fixed_y = 0;
      p.fixed_z = 0;
      p.timestamp = pose->timestamp;
      p.use_pose = false;
      t[i].push_back(p);
    }
  }
}

void MarkUsedPoses(vector <vector <SlamPose> > &traj, vector <bool> &optimize,
		   MatchList *matches, double inter_pose_dist)
{
  double d, last_x = 0, last_y = 0;
  Match *m;
  int i, j;

  for(i = 0; i < (int)traj.size(); i++) 
    if(optimize[i]) {
      /* use the first and last pose */
      traj[i][0].use_pose = true;
      traj[i][(int)traj[i].size() - 1].use_pose = 1;
      
      /* use a pose every N meters */
      for(j = 0; j < (int)traj[i].size(); j++) {
	d = hypot(traj[i][j].smooth_x - last_x, traj[i][j].smooth_y - last_y);
	if(j == 0 || d > inter_pose_dist) {
	  last_x = traj[i][j].smooth_x;
	  last_y = traj[i][j].smooth_y;
	  traj[i][j].use_pose = true;
	  d = 0;
	}
      }
    }

  /* mark poses that are used in matches */
  for(i = 0; i < matches->num_matches(); i++) {
    m = matches->match(i);
    if (optimize[m->tnum1])
      traj[m->tnum1][m->snum1].use_pose = 1;
    if (optimize[m->tnum2])
      traj[m->tnum2][m->snum2].use_pose = 1;
  }
}

void UpdateIndex(vector <SlamPose> &traj, dgc_velodyne_index *index_in,
		 dgc_velodyne_index *index_out)
{
  double u, dx1, dy1, dz1, dx2, dy2, dz2, offset_x, offset_y, offset_z;
  double utm_x, utm_y, first_x = 0, first_y = 0, first_z = 0;
  dgc_velodyne_index_pose *pose_in, *pose_out;
  int i, j, first = 1;
  char utmzone[10];

  // needed to get the UTM zone
  vlr::latLongToUtm(index_in->spin[0].pose[0].latitude,
	      index_in->spin[0].pose[0].longitude, 
	      &utm_x, &utm_y, utmzone);

  for(i = 0; i < index_in->num_spins; i++) {
    dx1 = traj[i].fixed_x - traj[i].smooth_x;
    dy1 = traj[i].fixed_y - traj[i].smooth_y;
    dz1 = traj[i].fixed_z - traj[i].smooth_z;
    if(i == index_in->num_spins - 1) {
      dx2 = dx1;
      dy2 = dy1;
      dz2 = dz1;
    }
    else {
      dx2 = traj[i + 1].fixed_x - traj[i + 1].smooth_x;
      dy2 = traj[i + 1].fixed_y - traj[i + 1].smooth_y;
      dz2 = traj[i + 1].fixed_z - traj[i + 1].smooth_z;
    }

    for(j = 0; j < index_in->spin[i].num_poses; j++) {
      if(i == index_in->num_spins - 1)
	u = 0;
      else
	u = (index_in->spin[i].pose[j].timestamp - 
	     index_in->spin[i].pose[0].timestamp) / 
	  (index_in->spin[i + 1].pose[0].timestamp - 
	   index_in->spin[i].pose[0].timestamp);
      offset_x = dx1 + u * (dx2 - dx1);
      offset_y = dy1 + u * (dy2 - dy1);
      offset_z = dz1 + u * (dz2 - dz1);

      pose_in = &(index_in->spin[i].pose[j]);
      pose_out = &(index_out->spin[i].pose[j]);

      vlr::utmToLatLong(pose_in->smooth_x + offset_x, pose_in->smooth_y + offset_y,
		  utmzone, &pose_out->latitude, &pose_out->longitude);
      pose_out->altitude = pose_in->smooth_z + offset_z;

      pose_out->smooth_x = pose_in->smooth_x + offset_x;
      pose_out->smooth_y = pose_in->smooth_y + offset_y;
      pose_out->smooth_z = pose_in->smooth_z + offset_z;

      if(first) {
	first_x = pose_out->smooth_x;
	first_y = pose_out->smooth_y;
	first_z = pose_out->smooth_z;
	first = 0;
      }

      pose_out->smooth_x -= first_x;
      pose_out->smooth_y -= first_y;
      pose_out->smooth_z -= first_z;
    }
  }
}

void SLAMCorrectIndexes(SlamInputs *inputs, MatchList *matches)
{
  vector <vector <SlamPose> > traj;
  vector <bool> optimize;
  vector <SlamFixedConstraint> fixed;
  vector <SlamMatchConstraint> match;

  dgc_velodyne_index_pose *pose1, *pose2;
  dgc_velodyne_index *newindex;
  double utm_x1, utm_y1, utm_z1;
  double utm_x2, utm_y2, utm_z2;
  char utmzone[10];
  SlamMatchConstraint mc;
  SlamFixedConstraint fc;
  Match *m;
  int i;

  fprintf(stderr, "Preparing SLAM problem... ");
  for (i = 0; i < inputs->num_logs(); i++) 
    optimize.push_back(inputs->log(i)->optimize());
  ExtractPoses(inputs, traj);
  MarkUsedPoses(traj, optimize, matches, 2.0);

  /* mark poses that are used in matches - this could be done faster */
  for(i = 0; i < matches->num_matches(); i++) {
    m = matches->match(i);
    pose1 = &(inputs->log(m->tnum1)->index()->spin[m->snum1].pose[0]);
    pose2 = &(inputs->log(m->tnum2)->index()->spin[m->snum2].pose[0]);

    if(optimize[m->tnum1] && optimize[m->tnum2]) {
      ConvertToUtm(pose1->latitude, pose1->longitude, pose1->altitude,
		   &utm_x1, &utm_y1, &utm_z1, utmzone);
      ConvertToUtm(pose2->latitude, pose2->longitude, pose2->altitude,
		   &utm_x2, &utm_y2, &utm_z2, utmzone);

      mc.pose1_ts = pose1->timestamp;
      mc.pose2_ts = pose2->timestamp;
      mc.dx = (utm_x2 - utm_x1) - m->offset[0][3];
      mc.dy = (utm_y2 - utm_y1) - m->offset[1][3];
      mc.dz = (utm_z2 - utm_z1) - m->offset[2][3];
      match.push_back(mc);
    }
    else if(optimize[m->tnum1]) {
      ConvertToUtm(pose1->latitude, pose1->longitude, pose1->altitude,
		   &utm_x1, &utm_y1, &utm_z1, utmzone);
      fc.x = utm_x1 + m->offset[0][3];
      fc.y = utm_y1 + m->offset[1][3];
      fc.z = utm_z1 + m->offset[2][3];
      fc.pose_ts = pose1->timestamp;  // this was pose2 before
      fixed.push_back(fc);
    }
    else if(optimize[m->tnum2]) {
      ConvertToUtm(pose2->latitude, pose2->longitude, pose2->altitude,
		   &utm_x2, &utm_y2, &utm_z2, utmzone);
      fc.x = utm_x2 - m->offset[0][3];
      fc.y = utm_y2 - m->offset[1][3];
      fc.z = utm_z2 - m->offset[2][3];
      fc.pose_ts = pose2->timestamp; // this was pose1 before
      fixed.push_back(fc);
    }
  }
  fprintf(stderr, "done.\n");

  fprintf(stderr, "Optimizing trajectory... ");
  double t1 = dgc_get_time();
  OptimizeTrajectories(traj, optimize, match, fixed, 
		       STD_ODOM, STD_GPS, STD_MATCH);
  double t2 = dgc_get_time();
  fprintf(stderr, "done. (%.2fs) \n", t2 - t1);

  fprintf(stderr, "Rewriting VLF indices.\n");
  for (i = 0; i < inputs->num_logs(); i++) {
    newindex = inputs->log(i)->corrected_index();
    UpdateIndex(traj[i], inputs->log(i)->index(), newindex);
    newindex->save(inputs->log(i)->corrected_index_filename());
  }
} 

