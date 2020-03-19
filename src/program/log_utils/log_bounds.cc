#include <roadrunner.h>
#include <applanix_interface.h>
#include <logio.h>
#include <lltransform.h>

using namespace dgc;

int main(int argc, char **argv)
{
  double min_latitude = 0, min_longitude = 0;
  double max_latitude = 0, max_longitude = 0;
  LineBuffer *line_buffer = NULL;
  ApplanixPose pose;
  double avg_lat, avg_lon, r;
  char *line = NULL;
  int first = 1;
  dgc_FILE *fp;

  /* interpet command line parameters */
  if(argc < 2)
    dgc_die("Error: not enough arguments\n"
            "Usage: %s logfile\n", argv[0]);

  fp = dgc_fopen(argv[1], "r");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for reading.\n", argv[1]);

  fprintf(stderr, "Reading logfile... ");
  line_buffer = new LineBuffer;
  do {
    line = line_buffer->ReadLine(fp);

    if(line != NULL) {
      if(strncmp(line, "APPLANIX_POSE_V1", 16) == 0) {
	StringV1ToApplanixPose(dgc_next_word(line), &pose);
	
	if(first || pose.latitude < min_latitude)
	  min_latitude = pose.latitude;
	if(first || pose.latitude > max_latitude)
	  max_latitude = pose.latitude;
	if(first || pose.longitude < min_longitude)
	  min_longitude = pose.longitude;
	if(first || pose.longitude > max_longitude)
	  max_longitude = pose.longitude;
	first = 0;
      }
      else if(strncmp(line, "APPLANIX_POSE_V2", 16) == 0) {
	StringV2ToApplanixPose(dgc_next_word(line), &pose);
	
	if(first || pose.latitude < min_latitude)
	  min_latitude = pose.latitude;
	if(first || pose.latitude > max_latitude)
	  max_latitude = pose.latitude;
	if(first || pose.longitude < min_longitude)
	  min_longitude = pose.longitude;
	if(first || pose.longitude > max_longitude)
	  max_longitude = pose.longitude;
	first = 0;
      }
    }
  } while(line != NULL);
  dgc_fclose(fp);
  fprintf(stderr, "done.\n");

  avg_lat = 0.5 * (min_latitude + max_latitude);
  avg_lon = 0.5 * (min_longitude + max_longitude);
  r = dgc_fmax(dgc_d2r(max_latitude - min_latitude) * cos(dgc_d2r(avg_lat)),
	       dgc_d2r(max_longitude - min_longitude)) * 6378100;
  fprintf(stderr, "\nLatitude: %.8f %.8f\n", min_latitude, max_latitude);
  fprintf(stderr, "Longitude: %.8f %.8f\n", min_longitude, max_longitude);
  fprintf(stderr, "Center: %.8f %.8f\n", avg_lat, avg_lon);
  fprintf(stderr, "Radius (approx) %.2f meters %.2f miles\n", r,
	  dgc_meters2miles(r));
  return 0;
}
