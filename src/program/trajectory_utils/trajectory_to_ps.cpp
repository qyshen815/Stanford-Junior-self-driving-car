#include <roadrunner.h>
#include <trajectory.h>
#include <pswrap.h>

int main(int argc, char **argv)
{
  dgc_trajectory_p trajectory;
  int i;
  double diff, min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
  ps_doc_p doc;

  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s file.traj\n", argv[0]);
  
  trajectory = dgc_trajectory_read(argv[1]);

  /* find bounding box */
  for(i = 0; i < trajectory->num_waypoints; i++) {
    if(trajectory->waypoint[i].x < min_x)
      min_x = trajectory->waypoint[i].x;
    else if(trajectory->waypoint[i].x > max_x)
      max_x = trajectory->waypoint[i].x;
    if(trajectory->waypoint[i].y < min_y)
      min_y = trajectory->waypoint[i].y;
    else if(trajectory->waypoint[i].y > max_y)
      max_y = trajectory->waypoint[i].y;
  }
  if(max_x - min_x > max_y - min_y) {
    diff = max_x - min_x - (max_y - min_y);
    max_y += diff / 2.0;
    min_y -= diff / 2.0;
  }
  else {
    diff = max_y - min_y - (max_x - min_x);
    max_x += diff / 2.0;
    min_x -= diff / 2.0;
  }
  diff = max_x - min_x;
  max_x += 0.2 * diff;
  min_x -= 0.2 * diff;
  max_y += 0.2 * diff;
  min_y -= 0.2 * diff;

  fprintf(stderr, "Drawing trajectory to path.ps\n");
  doc = ps_open("path.ps", 6, 6, 0);
  ps_set_color(doc, 0, 0, 0);
  for(i = 0; i < trajectory->num_waypoints - 1; i++) 
    ps_draw_line(doc,
                 (trajectory->waypoint[i].x - min_x) / (max_x - min_x) * 6.0,
                 (trajectory->waypoint[i].y - min_y) / (max_y - min_y) * 6.0,
                 (trajectory->waypoint[i + 1].x - min_x) / 
                 (max_x - min_x) * 6.0,
                 (trajectory->waypoint[i + 1].y - min_y) / 
                 (max_y - min_y) * 6.0);
  ps_draw_line(doc,
               (trajectory->waypoint[trajectory->num_waypoints - 1].x - min_x)
               / (max_x - min_x) * 6.0,
               (trajectory->waypoint[trajectory->num_waypoints - 1].y - min_y)
               / (max_y - min_y) * 6.0,
               (trajectory->waypoint[0].x - min_x) / (max_x - min_x) * 6.0,
               (trajectory->waypoint[0].y - min_y) / (max_y - min_y) * 6.0);

  ps_set_color(doc, 255, 255, 0);
  for(i = 0; i < trajectory->num_waypoints; i++) 
    ps_draw_circle(doc, 1, 
                   (trajectory->waypoint[i].x - min_x) / (max_x - min_x) * 6.0,
                   (trajectory->waypoint[i].y - min_y) / (max_y - min_y) * 6.0,
                   0.01);
  ps_close(doc);

  return 0;
}
