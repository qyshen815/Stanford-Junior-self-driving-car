#include "view.h"

vlr::PerceptionStopZones       stop_zones;

int
check_stop_zone( int segment, int lane, int waypoint )
{
  int  i;
  for (i=0; i<stop_zones.num_zones; i++) {
    if (stop_zones.zone[i].segment  == segment    &&
	stop_zones.zone[i].waypoint == waypoint  &&
	stop_zones.zone[i].lane     == lane) {
      return(stop_zones.zone[i].state);
    }
  }
  return(-1);
}

void
draw_stop_zones(vlr::rndf::RoadNetwork& /*rn*/, double /*origin_x*/, double /*origin_y*/)
{
// rndf_waypoint *w;
// int i, j, k, state;
// double heading;
//
// for(i = 0; i < rndf->num_segments(); i++) {
//   for(j = 0; j < rndf->segment(i)->num_lanes(); j++) {
//     for(k = 0; k < rndf->segment(i)->lane(j)->num_original_waypoints(); k++) {
//       w = rndf->segment(i)->lane(j)->original_waypoint(k);
//       heading = w->heading();
//       if(w->stop()) {
//         glPushMatrix();
//         glTranslatef(w->utm_x() - origin_x, w->utm_y() - origin_y, -0.05 );
//         glRotatef(dgc_r2d(heading), 0, 0, 1);
//	 state = check_stop_zone( i, j, k );
//	 if (state>=0) {
//	   switch(state) {
//	   case ZONE_STATE_UNKNOWN:
//	     glColor4f(0.5, 0.5, 0.5, 0.8);
//	     break;
//	   case ZONE_STATE_OCCUPIED:
//	     glColor4f(1.0, 0.0, 0.0, 0.8);
//	     break;
//	   case ZONE_STATE_FREE:
//	     glColor4f(0.0, 1.0, 0.0, 0.8);
//	     break;
//	   case ZONE_STATE_OCCLUDED:
//	     glColor4f(0.0, 0.0, 0.1, 0.8);
//	     break;
//	   case ZONE_STATE_OBSTACLE_MOVING:
//	   	 glColor4f(1.0, 0.4, 0.0, 0.8);
//	   	 break;
//	   case ZONE_STATE_OBSTACLE_STOPPED:
//	   	 glColor4f(0.8, 1.0, 0., 0.8);
//	   	 break;
//	   default:
//	     glColor4f(0.0, 0.0, 1.0, 0.3);
//	     break;
//	   }
//	 } else {
//	   glColor4f(0.0, 0.0, 1.0, 0.3);
//	 }
//	 glBegin(GL_QUADS);
//	 glVertex2f( stop_zones_dist_before_line,
//		     stop_zones_side_shift+stop_zones_width/2.0);
//	 glVertex2f( stop_zones_dist_before_line,
//		     stop_zones_side_shift-stop_zones_width/2.0);
//	 glVertex2f(-stop_zones_dist_behind_line,
//		    stop_zones_side_shift-stop_zones_width/2.0);
//	 glVertex2f(-stop_zones_dist_behind_line,
//		    stop_zones_side_shift+stop_zones_width/2.0);
//	 glEnd();
//         glPopMatrix();
//       }
//     }
//   }
// }
}

