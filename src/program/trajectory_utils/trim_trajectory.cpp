#include <roadrunner.h>
#include <trajectory.h>

void dgc_trajectory_trim_end(dgc_trajectory_p trajectory, double length)
{
  int i;
  double d;

  i = trajectory->num_waypoints - 1;
  d = 0;
  while(d < length) {
    i--;
    d = hypot(trajectory->waypoint[trajectory->num_waypoints - 1].x -
              trajectory->waypoint[i].x,
              trajectory->waypoint[trajectory->num_waypoints - 1].y - 
              trajectory->waypoint[i].y);
  }
  trajectory->num_waypoints = i;
}

void dgc_trajectory_trim_beginning(dgc_trajectory_p trajectory, double length)
{
  int i, j;
  double d;

  i = 0;
  d = 0;
  while(d < length) {
    i++;
    d = hypot(trajectory->waypoint[0].x - trajectory->waypoint[i].x,
              trajectory->waypoint[0].y - trajectory->waypoint[i].y);
  }
  for(j = i; j < trajectory->num_waypoints; j++)
    trajectory->waypoint[j - i] = trajectory->waypoint[j];
  trajectory->num_waypoints -= i;
}

int main(int argc, char **argv)
{
  char input_filename[256], output_filename[256];
  dgc_trajectory_p trajectory;
  double trim_length, trim_beginning;

  /* read input trajectory */
  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s trajectory-file trim-length [trim-beginning]\n", 
            argv[0]);

  strcpy(input_filename, argv[1]);
  strcpy(output_filename, argv[1]);
  dgc_trajectory_create_filename(input_filename, output_filename);

  trajectory = dgc_trajectory_read(input_filename);
  trim_length = atof(argv[2]);
  if(argc >= 4)
    trim_beginning = atof(argv[3]);
  else
    trim_beginning = 0;
  fprintf(stderr, "Trimming %.1f meters off end of trajectory.\n", 
          trim_length);
  dgc_trajectory_trim_end(trajectory, trim_length);
  if(trim_beginning > 0) {
    fprintf(stderr, "Trimming %.1f meters off beginning of trajectory.\n", 
            trim_beginning);
    dgc_trajectory_trim_beginning(trajectory, trim_beginning);
  }
  fprintf(stderr, "Writing output trajectory %s\n", output_filename);
  dgc_trajectory_write(trajectory, output_filename);
  return 0;
}
