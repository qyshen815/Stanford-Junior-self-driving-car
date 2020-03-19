#include <roadrunner.h>
#include <lltransform.h>

#define R_EARTH 6378100.0

int main(int argc, char **argv)
{
  FILE *fp;
  double lon, lat, r;

  if(argc < 5)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s lon lat radius-miles boundaryfile.txt\n", argv[0]);
  
  lon = atof(argv[1]);
  lat = atof(argv[2]);
  r = dgc_miles2meters(atof(argv[3]));


  fp = fopen(argv[4], "w");
  fprintf(fp, "%f\n", lat - dgc_r2d(r / R_EARTH));
  fprintf(fp, "%f\n", lon - dgc_r2d(r / R_EARTH / cos(dgc_d2r(lat))));
  fprintf(fp, "%f\n", lat + dgc_r2d(r / R_EARTH));
  fprintf(fp, "%f\n", lon + dgc_r2d(r / R_EARTH / cos(dgc_d2r(lat))));
  
  fclose(fp);
  return 0;
}
