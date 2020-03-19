#include <roadrunner.h>
#include <rndf.h>
#include <lltransform.h>
#include <transform.h>

#define       BASE_LAT           37.7728635
#define       BASE_LON         -122.2973709
#define       ROT_DEG                  (180-3.3)
#define       X_OFFSET                -103.0
#define       Y_OFFSET                 45.0

#define       INT_SIZE                 20.0
#define       LOOP_HEIGHT              60.0
#define       LOOP_WIDTH              140.0
#define       TURN_RADIUS               9.5
#define       STUB_LENGTH              40.0

using namespace dgc;
using namespace vlr;

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

  /* right side segment */
  segment = new rndf_segment;

  lane = add_n_waypoints(lw / 2.0, 
			 INT_SIZE / 2.0, 
			 lw / 2.0, 
			 LOOP_HEIGHT - TURN_RADIUS, utmzone, 3, lw);
  segment->append_lane(lane);
  delete lane;
  lane = add_n_waypoints(-lw / 2.0, 
			 LOOP_HEIGHT - TURN_RADIUS, 
			 -lw / 2.0, 
			 INT_SIZE / 2.0, utmzone, 3, lw);
  segment->append_lane(lane);
  delete lane;

  rndf->append_segment(segment);
  delete segment;

  /* top segment */
  segment = new rndf_segment;
  lane = add_n_waypoints(-TURN_RADIUS, 
			 LOOP_HEIGHT + lw / 2.0, 
			 -LOOP_WIDTH + TURN_RADIUS, 
			 LOOP_HEIGHT + lw / 2.0, utmzone, 7, lw);
  segment->append_lane(lane);
  lane = add_n_waypoints(-LOOP_WIDTH + TURN_RADIUS, 
			 LOOP_HEIGHT - lw / 2.0, 
			 -TURN_RADIUS, 
			 LOOP_HEIGHT - lw / 2.0, utmzone, 7, lw);
  segment->append_lane(lane);
  rndf->append_segment(segment);
  delete segment;

  /* left segment */
  segment = new rndf_segment;
  lane = add_n_waypoints(-LOOP_WIDTH - lw / 2.0, 
			 LOOP_HEIGHT - TURN_RADIUS,
			 -LOOP_WIDTH - lw / 2.0, 
			 TURN_RADIUS, utmzone, 3, lw);
  segment->append_lane(lane);
  lane = add_n_waypoints(-LOOP_WIDTH + lw / 2.0, 
			 TURN_RADIUS,
			 -LOOP_WIDTH + lw / 2.0, 
			 LOOP_HEIGHT - TURN_RADIUS, utmzone, 3, lw);
  segment->append_lane(lane);
  rndf->append_segment(segment);
  delete segment;

  /* bottom segment */
  segment = new rndf_segment;
  lane = add_n_waypoints(-LOOP_WIDTH + TURN_RADIUS, 
			 -lw / 2.0,
			 -INT_SIZE / 2.0, 
			 -lw / 2.0, utmzone, 7, lw);
  segment->append_lane(lane);
  lane = add_n_waypoints(-INT_SIZE / 2.0, 
			 lw / 2.0,
			 -LOOP_WIDTH + TURN_RADIUS, 
			 lw / 2.0, utmzone, 7, lw);
  segment->append_lane(lane);
  rndf->append_segment(segment);
  delete segment;

  /* bottom stub */
  segment = new rndf_segment;
  lane =add_n_waypoints(-lw / 2.0,
			-INT_SIZE / 2.0,
			-lw / 2.0,
			-INT_SIZE / 2.0 - STUB_LENGTH, utmzone, 4, lw);
  segment->append_lane(lane);
  lane = add_n_waypoints(lw / 2.0,
			 -INT_SIZE / 2.0 - STUB_LENGTH, 
			 lw / 2.0,
			 -INT_SIZE / 2.0, utmzone, 4, lw);
  segment->append_lane(lane);
  rndf->append_segment(segment);
  delete segment;

  /* right stub */
  segment = new rndf_segment;
  lane = add_n_waypoints(INT_SIZE / 2.0, 
			 -lw / 2.0,
			 INT_SIZE / 2.0 + STUB_LENGTH, 
			 -lw / 2.0, utmzone, 4, lw);
  segment->append_lane(lane);
  lane = add_n_waypoints(INT_SIZE / 2.0 + STUB_LENGTH, 
			 lw / 2.0,
			 INT_SIZE / 2.0, 
			 lw / 2.0, utmzone, 4, lw);
  segment->append_lane(lane);
  rndf->append_segment(segment);
  delete segment;

#ifdef blah
  /* top */


  /* left side */

  /* bottom */

  segment->append_lane(lane);
  delete lane;

  lane = new rndf_lane;
  lane->width(lw);
  lane->left_boundary(double_yellow);

  /* bottom */

  /* left side */

  /* top */


  segment->append_lane(lane);
  delete lane;

  rndf->append_segment(segment);
  delete segment;


  /* bottom stub */
  
  segment = new rndf_segment;
  
  lane = new rndf_lane;
  lane->width(lw);
  lane->left_boundary(double_yellow);


  segment->append_lane(lane);
  delete lane;

  lane = new rndf_lane;
  lane->width(lw);
  lane->left_boundary(double_yellow);


  segment->append_lane(lane);
  delete lane;

  rndf->append_segment(segment);
  delete segment;

  /* right stub */

  segment = new rndf_segment;
  
  lane = new rndf_lane;
  lane->width(lw);
  lane->left_boundary(double_yellow);

  segment->append_lane(lane);
  delete lane;

  lane = new rndf_lane;
  lane->width(lw);
  lane->left_boundary(double_yellow);


  segment->append_lane(lane);
  delete lane;

  rndf->append_segment(segment);
  delete segment;

#endif

  transform_rndf(rndf, base_x, base_y, dgc_d2r(ROT_DEG));

  rndf->save("alamedarndf.txt", false);

  return 0;
}
