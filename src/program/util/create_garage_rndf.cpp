#include <roadrunner.h>
#include <lltransform.h>
#include <rndf.h>
#include <transform.h>

#define       BASE_LAT            37.4317147
#define       BASE_LON          -122.1820519
#define       ROT_DEG                   +32.5
#define       X_OFFSET                  0
#define       Y_OFFSET                  0

#define       LOOP_HEIGHT              80.0
#define       LOOP_WIDTH               90.0
#define       TURN_RADIUS               9.5

using namespace dgc;

rndf_lane *add_n_waypoints(double x1, double y1, double x2, double y2, 
			   char *utmzone, int n, double lw)
{
  rndf_waypoint *w;
  rndf_lane *lane;
  double u;
  int i;

  lane = new rndf_lane;
  lane->width(lw);
  lane->left_boundary(double_yellow);

  w = new rndf_waypoint;
  w->original(true);
  for(i = 0; i < n; i++) {
    u = i / (double)(n - 1);
    w->set_utm(x1 + u * (x2 - x1), y1 + u * (y2 - y1), utmzone);
    lane->append_waypoint(w);
  }
  delete w;
  return lane;
}

void transform_rndf(rndf_file *rndf, double x_offset, double y_offset,
		    double theta_offset)
{
  dgc_transform_t t;
  double x, y, z;
  int i, j, k;
  rndf_waypoint *w;

  dgc_transform_identity(t);
  dgc_transform_rotate_z(t, theta_offset);
  dgc_transform_translate(t, x_offset, y_offset, 0);
  dgc_transform_translate(t, X_OFFSET, Y_OFFSET, 0);
  
  for(i = 0; i < rndf->num_segments(); i++)
    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
      for(k = 0; k < rndf->segment(i)->lane(j)->num_waypoints(); k++) {
	w = rndf->segment(i)->lane(j)->waypoint(k);
	x = w->utm_x();
	y = w->utm_y();
	z = 0;
	dgc_transform_point(&x, &y, &z, t);
	w->set_utm(x, y, w->utmzone());
      }
}

int main(void)
{
  rndf_file *rndf = NULL;
  rndf_lane *lane = NULL;
  rndf_segment *segment = NULL;
  char utmzone[10];

  double lw = dgc_feet2meters(15.0);
  double base_x, base_y;

  rndf = new rndf_file;

  vlr::latLongToUtm(BASE_LAT, BASE_LON, &base_x, &base_y, utmzone);

  segment = new rndf_segment;




  /* right side segment */
  segment = new rndf_segment;

  lane = add_n_waypoints(LOOP_WIDTH / 2.0 + lw / 2, -LOOP_HEIGHT / 2.0 + TURN_RADIUS,
			 LOOP_WIDTH / 2.0 + lw / 2, LOOP_HEIGHT / 2.0 - TURN_RADIUS,
			 utmzone, 3, lw);
  segment->append_lane(lane);
  delete lane;

  rndf->append_segment(segment);
  delete segment;

  segment = new rndf_segment;

  lane = add_n_waypoints(LOOP_WIDTH / 2.0 - TURN_RADIUS, LOOP_HEIGHT / 2.0 + lw / 2,
			 -LOOP_WIDTH / 2.0 + TURN_RADIUS, LOOP_HEIGHT / 2.0 + lw / 2,
			 utmzone, 3, lw);
  segment->append_lane(lane);
  delete lane;

  rndf->append_segment(segment);
  delete segment;


  segment = new rndf_segment;

  lane = add_n_waypoints(-LOOP_WIDTH / 2.0 - lw / 2, LOOP_HEIGHT / 2.0 - TURN_RADIUS,
			 -LOOP_WIDTH / 2.0 - lw / 2, -LOOP_HEIGHT / 2.0 + TURN_RADIUS,
			 utmzone, 3, lw);
  segment->append_lane(lane);
  delete lane;

  rndf->append_segment(segment);
  delete segment;


  segment = new rndf_segment;

  lane = add_n_waypoints(-LOOP_WIDTH / 2.0 + TURN_RADIUS, -LOOP_HEIGHT / 2.0 - lw / 2,
			 LOOP_WIDTH / 2.0 - TURN_RADIUS, -LOOP_HEIGHT / 2.0 - lw / 2,
			 utmzone, 3, lw);
  segment->append_lane(lane);
  delete lane;

  rndf->append_segment(segment);
  delete segment;

  transform_rndf(rndf, base_x, base_y, dgc_d2r(ROT_DEG));

  rndf->save("garage_rndf.txt", false);

  return 0;
}
