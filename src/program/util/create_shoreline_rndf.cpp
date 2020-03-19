#include <roadrunner.h>
#include <rndf.h>
#include <lltransform.h>
#include <transform.h>

#define       BASE_LAT            37.4278227
#define       BASE_LON          -122.0761790
#define       ROT_DEG                   +82
#define       X_OFFSET                  -48.0
#define       Y_OFFSET                  -20.0

/*
#define       BASE_LAT            37.4278227
#define       BASE_LON          -122.0761790
#define       ROT_DEG                   +80
#define       X_OFFSET                  -67.0
#define       Y_OFFSET                  -40.0
*/

#define       INT_SIZE                 20.0
#define       LOOP_HEIGHT              60.0
#define       LOOP_WIDTH              145.0
#define       TURN_RADIUS               9.5
#define       STUB_LENGTH              40.0

using namespace dgc;

rndf_lane *add_n_waypoints(rndf_lane *lane, double x1, double y1, 
			   double x2, double y2, 
			   char *utmzone, int n, double lw)
{
  rndf_waypoint *w;
  double u;
  int i;

  if(lane == NULL) {
    lane = new rndf_lane;
    lane->width(lw);
    lane->left_boundary(double_yellow);
  }

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

  for(i = 0; i < rndf->num_zones(); i++)
    for(j = 0; j < rndf->zone(i)->num_perimeter_points(); j++) {
      w = rndf->zone(i)->perimeter(j);
      x = w->utm_x();
      y = w->utm_y();
      z = 0;
      dgc_transform_point(&x, &y, &z, t);
      w->set_utm(x, y, w->utmzone());
    }
}

void shoreline_right_rndf(void)
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

  lane = add_n_waypoints(NULL, lw / 2.0,
			 -INT_SIZE / 2.0 - STUB_LENGTH, 
			 lw / 2.0,
			 -INT_SIZE / 2.0, utmzone, 4, lw);
  add_n_waypoints(lane, lw / 2.0, 
		  INT_SIZE / 2.0, 
		  lw / 2.0, 
		  LOOP_HEIGHT - TURN_RADIUS, utmzone, 3, lw);
  add_n_waypoints(lane, -TURN_RADIUS, 
		  LOOP_HEIGHT + lw / 2.0, 
		  -LOOP_WIDTH + TURN_RADIUS, 
		  LOOP_HEIGHT + lw / 2.0, utmzone, 7, lw);
  add_n_waypoints(lane, -LOOP_WIDTH - lw / 2.0, 
		  LOOP_HEIGHT - TURN_RADIUS,
		  -LOOP_WIDTH - lw / 2.0, 
		  TURN_RADIUS, utmzone, 3, lw);
  add_n_waypoints(lane, -LOOP_WIDTH + TURN_RADIUS, 
		  -lw / 2.0,
		  -INT_SIZE / 2.0, 
		  -lw / 2.0, utmzone, 7, lw);

  segment->append_lane(lane);
  delete lane;
  lane = add_n_waypoints(NULL, -INT_SIZE / 2.0, 
			 lw / 2.0,
			 -LOOP_WIDTH + TURN_RADIUS, 
			 lw / 2.0, utmzone, 7, lw);
  add_n_waypoints(lane, -LOOP_WIDTH + lw / 2.0, 
		  TURN_RADIUS,
		  -LOOP_WIDTH + lw / 2.0, 
		  LOOP_HEIGHT - TURN_RADIUS, utmzone, 3, lw);
  add_n_waypoints(lane, -LOOP_WIDTH + TURN_RADIUS, 
		  LOOP_HEIGHT - lw / 2.0, 
		  -TURN_RADIUS, 
		  LOOP_HEIGHT - lw / 2.0, utmzone, 7, lw);
  add_n_waypoints(lane, -lw / 2.0, 
		  LOOP_HEIGHT - TURN_RADIUS, 
		  -lw / 2.0, 
		  INT_SIZE / 2.0, utmzone, 3, lw);

  add_n_waypoints(lane, -lw / 2.0,
		  -INT_SIZE / 2.0,
		  -lw / 2.0,
		  -INT_SIZE / 2.0 - STUB_LENGTH, utmzone, 4, lw);


  segment->append_lane(lane);
  delete lane;

  rndf->append_segment(segment);
  delete segment;

#ifdef blah
  /* bottom stub */
  segment = new rndf_segment;
  lane = add_n_waypoints(NULL, -lw / 2.0,
			-INT_SIZE / 2.0,
			-lw / 2.0,
			-INT_SIZE / 2.0 - STUB_LENGTH, utmzone, 4, lw);
  segment->append_lane(lane);
  delete lane;


  segment->append_lane(lane);
  delete lane;

  rndf->append_segment(segment);
  delete segment;
#endif

  /* right stub */
  segment = new rndf_segment;
  lane = add_n_waypoints(NULL, INT_SIZE / 2.0, 
			 -lw / 2.0,
			 INT_SIZE / 2.0 + STUB_LENGTH, 
			 -lw / 2.0, utmzone, 4, lw);
  segment->append_lane(lane);
  delete lane;

  lane = add_n_waypoints(NULL, INT_SIZE / 2.0 + STUB_LENGTH, 
			 lw / 2.0,
			 INT_SIZE / 2.0, 
			 lw / 2.0, utmzone, 4, lw);
  segment->append_lane(lane);
  delete lane;

  rndf->append_segment(segment);
  delete segment;

  transform_rndf(rndf, base_x, base_y, dgc_d2r(ROT_DEG));
  rndf->save("shoreline_right_rndf.txt", false);
}

void shoreline_left_rndf(void)
{
  rndf_file *rndf = NULL;
  rndf_lane *lane = NULL;
  rndf_segment *segment = NULL;
  rndf_zone *zone = NULL;
  rndf_waypoint *w = NULL;
  char utmzone[10];

  double lw = dgc_feet2meters(15.0);
  double base_x, base_y;

  rndf = new rndf_file;

  vlr::latLongToUtm(BASE_LAT, BASE_LON, &base_x, &base_y, utmzone);

  /* right side segment */
  segment = new rndf_segment;

  lane = add_n_waypoints(NULL, lw / 2.0, 
			 INT_SIZE / 2.0, 
			 lw / 2.0, 
			 LOOP_HEIGHT - TURN_RADIUS, utmzone, 3, lw);

  add_n_waypoints(lane, -TURN_RADIUS, 
		  LOOP_HEIGHT + lw / 2.0, 
		  -LOOP_WIDTH / 2.0 + INT_SIZE / 2.0,
		  LOOP_HEIGHT + lw / 2.0, utmzone, 3, lw);
  add_n_waypoints(lane, 
		  -LOOP_WIDTH / 2 - INT_SIZE / 2.0,
		  LOOP_HEIGHT + lw / 2.0, 
		  -LOOP_WIDTH + TURN_RADIUS, 
		  LOOP_HEIGHT + lw / 2.0, utmzone, 3, lw);
  add_n_waypoints(lane, -LOOP_WIDTH - lw / 2.0, 
		  LOOP_HEIGHT - TURN_RADIUS,
		  -LOOP_WIDTH - lw / 2.0, 
		  TURN_RADIUS, utmzone, 3, lw);
  add_n_waypoints(lane, -LOOP_WIDTH + TURN_RADIUS, 
		  -lw / 2.0,
		  -LOOP_WIDTH / 2.0 - INT_SIZE / 2.0, 
		  -lw / 2.0, utmzone, 3, lw);
  add_n_waypoints(lane, 
		  -LOOP_WIDTH / 2.0 + INT_SIZE / 2.0,
		  -lw / 2.0,
		  -INT_SIZE / 2.0, 
		  -lw / 2.0, utmzone, 3, lw);
  segment->append_lane(lane);
  delete lane;

  lane = add_n_waypoints(NULL,
			 -INT_SIZE / 2.0, 
			 lw / 2.0,
			 -LOOP_WIDTH / 2 + INT_SIZE / 2,
			 lw / 2.0, utmzone, 3, lw);
  add_n_waypoints(lane, 
		  -LOOP_WIDTH / 2 - INT_SIZE / 2.0, 
		  lw / 2.0,
		  -LOOP_WIDTH + TURN_RADIUS, 
		  lw / 2.0, utmzone, 3, lw);
  add_n_waypoints(lane, -LOOP_WIDTH + lw / 2.0, 
		  TURN_RADIUS,
		  -LOOP_WIDTH + lw / 2.0, 
		  LOOP_HEIGHT - TURN_RADIUS, utmzone, 3, lw);
  add_n_waypoints(lane, -LOOP_WIDTH + TURN_RADIUS, 
		  LOOP_HEIGHT - lw / 2.0, 
		  -LOOP_WIDTH / 2.0 - INT_SIZE / 2.0,
		  LOOP_HEIGHT - lw / 2.0, utmzone, 3, lw);
  add_n_waypoints(lane, 
		  -LOOP_WIDTH / 2.0 + INT_SIZE / 2.0,
		  LOOP_HEIGHT - lw / 2.0, 
		  -TURN_RADIUS, 
		  LOOP_HEIGHT - lw / 2.0, utmzone, 3, lw);
  add_n_waypoints(lane, -lw / 2.0, 
		  LOOP_HEIGHT - TURN_RADIUS, 
		  -lw / 2.0, 
		  INT_SIZE / 2.0, utmzone, 3, lw);
  segment->append_lane(lane);
  delete lane;

  rndf->append_segment(segment);
  delete segment;

  /* center road */
  segment = new rndf_segment;

  lane = add_n_waypoints(NULL, 
			 -LOOP_WIDTH / 2.0 + lw / 2.0, INT_SIZE / 2.0,
			 -LOOP_WIDTH / 2.0 + lw / 2.0, 
			 LOOP_HEIGHT - INT_SIZE / 2.0, utmzone, 3, lw);
  segment->append_lane(lane);
  delete lane;

  lane = add_n_waypoints(NULL, 
			 -LOOP_WIDTH / 2.0 - lw / 2.0, 
			 LOOP_HEIGHT - INT_SIZE / 2.0, 
			 -LOOP_WIDTH / 2.0 - lw / 2.0, 
			 INT_SIZE / 2.0,

			 utmzone, 3, lw);
  segment->append_lane(lane);
  delete lane;

  rndf->append_segment(segment);
  delete segment;

  /* bottom stub */
  segment = new rndf_segment;
  lane = add_n_waypoints(NULL, -lw / 2.0,
			-INT_SIZE / 2.0,
			-lw / 2.0,
			-INT_SIZE / 2.0 - STUB_LENGTH, utmzone, 4, lw);
  segment->append_lane(lane);
  delete lane;

  lane = add_n_waypoints(NULL, lw / 2.0,
			 -INT_SIZE / 2.0 - STUB_LENGTH, 
			 lw / 2.0,
			 -INT_SIZE / 2.0, utmzone, 4, lw);
  segment->append_lane(lane);
  delete lane;

  rndf->append_segment(segment);
  delete segment;

  /* right stub */
  segment = new rndf_segment;
  lane = add_n_waypoints(NULL, INT_SIZE / 2.0, 
			 -lw / 2.0,
			 INT_SIZE / 2.0 + STUB_LENGTH, 
			 -lw / 2.0, utmzone, 4, lw);
  segment->append_lane(lane);
  delete lane;

  lane = add_n_waypoints(NULL, INT_SIZE / 2.0 + STUB_LENGTH, 
			 lw / 2.0,
			 INT_SIZE / 2.0, 
			 lw / 2.0, utmzone, 4, lw);
  segment->append_lane(lane);
  delete lane;

  rndf->append_segment(segment);
  delete segment;

  zone = new rndf_zone;
  
  w = new rndf_waypoint;
  w->original(true);

  w->set_utm(-LOOP_WIDTH + TURN_RADIUS, TURN_RADIUS, utmzone);
  zone->append_perimeter_point(w);

  w->set_utm(-LOOP_WIDTH + TURN_RADIUS, LOOP_HEIGHT * 0.4, utmzone);
  zone->append_perimeter_point(w);
 
  w->set_utm(-LOOP_WIDTH + TURN_RADIUS, LOOP_HEIGHT - TURN_RADIUS, utmzone);
  zone->append_perimeter_point(w);

  w->set_utm(-LOOP_WIDTH / 2 - TURN_RADIUS, LOOP_HEIGHT - TURN_RADIUS, utmzone);
  zone->append_perimeter_point(w);

  w->set_utm(-LOOP_WIDTH / 2 - TURN_RADIUS, LOOP_HEIGHT * 0.4, utmzone);
  zone->append_perimeter_point(w);

  w->set_utm(-LOOP_WIDTH / 2 - TURN_RADIUS, TURN_RADIUS, utmzone);
  zone->append_perimeter_point(w);

  delete w;

  rndf->append_zone(zone);

  transform_rndf(rndf, base_x, base_y, dgc_d2r(ROT_DEG));
  rndf->save("shoreline_left_rndf.txt", false);
}

int main(void)
{
  shoreline_right_rndf();
  shoreline_left_rndf();

  return 0;
}
