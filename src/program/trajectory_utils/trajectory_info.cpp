#include <roadrunner.h>
#include <trajectory.h>

int main(int argc, char **argv)
{
  dgc_trajectory_p t1;
  int i;
  double d, s;
  int h, m;

  if(argc < 2)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s trajectoryfile\n", argv[0]);

  t1 = dgc_trajectory_read(argv[1]);
  if(t1 == NULL)
    exit(1);
  
  d = 0;
  for(i = 0; i < t1->num_waypoints - 1; i++) 
    d += hypot(t1->waypoint[i + 1].x - t1->waypoint[i].x,
               t1->waypoint[i + 1].y - t1->waypoint[i].y);
  fprintf(stderr, "Trajectory length : %.2f m : %.1f km : %.1f mi\n", d,
          d / 1000.0, dgc_meters2miles(d));

  s = 0;
  for(i = 0; i < t1->num_waypoints - 1; i++)
    if(t1->waypoint[i].velocity > 0) {
      d = hypot(t1->waypoint[i + 1].x - t1->waypoint[i].x,
                t1->waypoint[i + 1].y - t1->waypoint[i].y);
      s += d / t1->waypoint[i].velocity;
    }
  
  h = (int)floor(s / 3600.0);
  s -= h * 3600.0;
  m = (int)floor(s / 60.0);
  s -= m * 60.0;

  fprintf(stderr, 
          "Minimum trajectory completion time : %d hours %d minutes %.2f seconds.\n", h, m, s);
  
  return 0;
}
