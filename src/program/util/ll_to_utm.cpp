#include <roadrunner.h>
#include <lltransform.h>

int main(int argc, char **argv)
{
  double lat, lon;
  double utm_x, utm_y;
  char utmzone[10];

  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s lat lon\n", argv[0]);
  lat = atof(argv[1]);
  lon = atof(argv[2]);
  vlr::latLongToUtm(lat, lon, &utm_x, &utm_y, utmzone);
  fprintf(stderr, "Lat/Lon %f, %f -> UTM %.2f %.2f %s\n", lat, lon, utm_x, utm_y, utmzone);
  return 0;
}
