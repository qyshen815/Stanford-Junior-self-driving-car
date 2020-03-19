#include <strings.h>
#include <wordexp.h>
#include "view.h"

dgc_rgb_t crgb[NUM_COLORS] = {
  { 1.0, 1.0, 1.0 },
  { 1.0, 0.0, 0.0 },
  { 0.0, 1.0, 0.0 },
  { 0.0, 0.0, 1.0 },
  { 1.0, 1.0, 0.0 },
  { 0.0, 1.0, 1.0 },
  { 1.0, 0.0, 1.0 },
  { 0.0, 0.0, 0.0 }
};

int
cycle_colors( int offset )
{
  return((change_colors+offset)%NUM_COLORS);
}

char *
createTimeString( double time )
{
  static char str[256];
  long tv_sec = (long) time;
  struct tm *actual_date;
  actual_date = localtime( &tv_sec );
  snprintf( str, 256, "%02d:%02d:%02d",
	    actual_date->tm_hour,
	    actual_date->tm_min,
	    actual_date->tm_sec );
  return(str);
}

char *
createDateString( double time )
{
  static char str[256];
  long tv_sec = (long) time;
  struct tm *actual_date;
  actual_date = localtime( &tv_sec );
  snprintf( str, 256, "%04d-%02d-%02d",
	    1900+actual_date->tm_year,
	    actual_date->tm_mon+1,
	    actual_date->tm_mday );
  return(str);
}

dgc_pose_t
dgc_applanix_pose_difference( ApplanixPose a, 
			      ApplanixPose b )
{
  dgc_pose_t  delta;
  delta.x      = a.smooth_x - b.smooth_x;
  delta.y      = a.smooth_y - b.smooth_y;
  delta.z      = a.smooth_z - b.smooth_z;
  delta.yaw    = dgc_normalize_theta(a.yaw   - b.yaw);
  delta.pitch  = dgc_normalize_theta(a.pitch - b.pitch);
  delta.roll   = dgc_normalize_theta(a.roll  - b.roll);
  return(delta);
}


dgc_rgb_t
dgc_hsv_to_rgb( dgc_hsv_t color )
{
   dgc_rgb_t   ret;
   int         i;
   double      aa, bb, cc, f;

  if (color.s == 0)
    ret.r = ret.g = ret.b = color.v;
  else {
    if (color.h == 1.0)
      color.h = 0;
    color.h *= 6.0;
    i = (int) floor (color.h);
    f = color.h - i;
    aa = color.v * (1 - color.s);
    bb = color.v * (1 - (color.s * f));
    cc = color.v * (1 - (color.s * (1 - f)));
    switch (i) {
    case 0:
      ret.r = color.v;
      ret.g = cc;
      ret.b = aa;
      break;
    case 1:
      ret.r = bb;
      ret.g = color.v;
      ret.b = aa;
      break;
    case 2:
      ret.r = aa;
      ret.g = color.v;
      ret.b = cc;
      break;
    case 3:
      ret.r = aa;
      ret.g = bb;
      ret.b = color.v;
      break;
    case 4:
      ret.r = cc;
      ret.g = aa;
      ret.b = color.v;
      break;
    case 5:
      ret.r = color.v;
      ret.g = aa;
      ret.b = bb;
      break;
    }
  }
  return(ret);
}

dgc_rgb_t
dgc_val_to_rgb( double val )
{
  dgc_hsv_t color = {1.0, 1.0, 1.0};

  /* cut to range [0.0,1.0] */
  val = MIN( 1.0, MAX( 0.0, val ) );

  /* the gradient is done by changing hue between blue and yellow */
  if (val>0.1) {
    color.h = fmod(0.555*val+.66666666,1.0);
  } else {
    /* if the val is smaller than 10% */
    color.h = .66666666;
    color.s = 1.0;
    color.v = val * 10.0;
  }
  
  return( dgc_hsv_to_rgb( color ) );
}

double 
beam_dist( dgc_velodyne_point_p pt1, dgc_velodyne_point_p pt2 )
{
  int dx = pt1->x - pt2->x;
  int dy = pt1->y - pt2->y;
  int dz = pt1->z - pt2->z;
  return(sqrt(dx*dx+dy*dy+dz*dz));
}

float
pts_dist( velodyne_pt_t pt1, velodyne_pt_t pt2 )
{
  float dx = (pt1.x - pt2.x);
  float dy = (pt1.y - pt2.y);
  float dz = (pt1.z - pt2.z);
  return(sqrt(dx*dx+dy*dy+dz*dz));
}

double 
pose_dist( dgc_pose_p pt1, dgc_pose_p pt2 )
{
  double dx = pt1->x - pt2->x;
  double dy = pt1->y - pt2->y;
  double dz = pt1->z - pt2->z;
  return(sqrt(dx*dx+dy*dy+dz*dz));
}

