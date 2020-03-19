#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <cmath>

#include <lltransform.h>

using namespace vlr;

int main(int argc, char **argv) {

  double cx, cy, phi=0;
  double r = 35;
  char zone[4];
  double c_lat = 37.4275144;
  double c_lon = -122.0769586;
  latLongToUtm(c_lat, c_lon, &cx,  &cy, zone);
  uint32_t num_points = int(2*M_PI*r);
  double phi_step = 2*M_PI/(num_points);

  FILE* rndf=NULL;

  char* rndf_name;
  char def_name[] = "circle_rndf.txt";

  if(argc>1) {
    rndf_name = argv[1];
  }
  else {
    rndf_name = def_name;
  }

  printf("Output file is %s\n", rndf_name);

  rndf=fopen(rndf_name, "w");
  if(!rndf) {
    printf("Could not open output file.\n");
  return -5;
  }

  if(argc>3) {
    c_lat = atof(argv[2]);
    c_lon = atof(argv[3]);
  }
  else {
    rndf=fopen("circle_rndf.txt", "w");
  }

  printf("Center coordinates: %.16lf, %.16lf\n", c_lat, c_lon);

  fprintf(rndf, "RNDF_name %s\n", rndf_name);
  fprintf(rndf, "num_segments 1\n");
  fprintf(rndf, "num_zones 0\n");
  fprintf(rndf, "num_intersections 0\n");
  fprintf(rndf, "format_version  1.1\n");
  fprintf(rndf, "creation_date 11/18/2009\n");
  fprintf(rndf, "segment 1\n");
  fprintf(rndf, "num_lanes 1\n");
  fprintf(rndf, "num_crosswalks  1\n");
  fprintf(rndf, "lane  1.1\n");
  fprintf(rndf, "num_waypoints %u\n", num_points);
  fprintf(rndf, "lane_width  12\n");
  fprintf(rndf, "exit  1.1.%u 1.1.1\n", num_points);
  fprintf(rndf, "stop  1.1.%u\n", num_points);
  fprintf(rndf, "checkpoint 1.1.90 1\n");

  for(uint32_t i = 0; i<num_points; i++, phi+=phi_step) {
    double x = cx + r*cos(phi);
    double y = cy + r*sin(phi);
    double lat, lon;
    utmToLatLong(x, y, zone, &lat, &lon);
    fprintf(rndf, "1.1.%u %.16lf %.16lf\n", i+1, lat, lon);
  }

  fprintf(rndf, "end_lane\n");
  fprintf(rndf, "end_segment\n");
  fprintf(rndf, "end_file\n");
  fclose(rndf);

return 0;
}
