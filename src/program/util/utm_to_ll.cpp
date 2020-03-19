#include <roadrunner.h>
#include <lltransform.h>

int main(int argc, char **argv)
{
  double lat, lon;
  double utm_x, utm_y;

  if(argc < 4)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s utm_x utm_y utmzone\n", argv[0]);
  utm_x = atof(argv[1]);
  utm_y = atof(argv[2]);
  vlr::utmToLatLong(utm_x, utm_y, argv[3], &lat, &lon);
  fprintf(stderr, "UTM %.2f %.2f %s -> Lat/Lon %.7f, %.7f\n", utm_x, utm_y, argv[3], lat, lon);
  return 0;
}
