#include <roadrunner.h>
#include <lltransform.h>

int main(int argc, char **argv)
{
  double d1, m1, s1, d2, m2, s2;
  double lat, lon;
  double utm_x, utm_y;
  char utmzone[10];

  if(argc < 7)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s d m s d m s\n", argv[0]);

  d1 = atof(argv[1]);
  m1 = atof(argv[2]);
  s1 = atof(argv[3]);

  d2 = atof(argv[4]);
  m2 = atof(argv[5]);
  s2 = atof(argv[6]);

  lat = d1 + m1 / 60.0 + s1 / 3600.0;
  lon = d2 + m2 / 60.0 + s2 / 3600.0;

  vlr::latLongToUtm(lat, lon, &utm_x, &utm_y, utmzone);
  fprintf(stderr, "Lat/Lon %f, %f -> UTM %.2f %.2f %s\n", lat, lon,
          utm_x, utm_y, utmzone);

  return 0;
}
