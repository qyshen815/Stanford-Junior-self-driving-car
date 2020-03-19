#ifndef LLTRANSFORM_H_
#define LLTRANSFORM_H_

#include <string>

namespace vlr {

typedef struct {
  double lat;
  double lon;
} coordinate_latlon_t;

typedef struct {
  double x;
  double y;
  char zone[4];
} coordinate_utm_t;

void latLongToUtm(double Lat, double Long, double* UTMEasting, double* UTMNorthing, std::string& UTMZone);
void latLongToUtm(double Lat, double Long, double* UTMEasting, double* UTMNorthing, char* UTMZone);
void utmToLatLong(double UTMEasting, double UTMNorthing, const std::string& UTMZone, double* Lat,  double* Long);
void utmToLatLong(double UTMEasting, double UTMNorthing, const char* UTMZone, double* Lat,  double* Long);

int spcsInit(int zone, int verbose, int use_nad27);
int spcsToLatLong(double easting, double northing, double *latitude, double *longitude);
int latLongToSpcs(double latitude, double longitude, double* easting, double* northing);

}

#endif
