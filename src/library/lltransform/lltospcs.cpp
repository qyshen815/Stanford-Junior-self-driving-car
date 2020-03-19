#include <roadrunner.h>
#include <projects.h>

namespace vlr {

static int spcs_initialized = 0;
static PJ *Proj = NULL;

int spcsInit(int zone, int verbose, int use_nad27) {
  char *pargv[10];

  pargv[0] = (char*)calloc(100, 1);
  strcpy(pargv[0], "+units=m");
  pargv[1] = (char*)calloc(100, 1);
  if (use_nad27)
    sprintf(pargv[1], "+init=/usr/share/proj/nad27:%d", zone);
  else
    sprintf(pargv[1], "+init=/usr/share/proj/nad83:%d", zone);
  if (!(Proj = pj_init(2, pargv))) {
    printf("Error: could not initialize conversion library.\n");
    return -1;
  }
  if (verbose) pj_pr_list(Proj);
  spcs_initialized = 1;
  return 0;
}

int spcsToLatLong(double easting, double northing, double *latitude, double *longitude) {
  projUV data;

  if (!spcs_initialized) spcsInit(405, 0, 0);
  data.u = easting;
  data.v = northing;
  data = pj_inv(data, Proj);
  *longitude = dgc_r2d(data.u);
  *latitude = dgc_r2d(data.v);
  if (data.u == HUGE_VAL)
    return -1;
  else
    return 0;
}

int latLongToSpcs(double latitude, double longitude, double* easting, double* northing) {
  projUV data;

  if (!spcs_initialized) spcsInit(405, 0, 0);
  data.u = longitude * M_PI / 180.0;
  data.v = latitude * M_PI / 180.0;
  data = pj_fwd(data, Proj);
  *easting = data.u;
  *northing = data.v;
  if (data.u == HUGE_VAL)
    return -1;
  else
    return 0;
}

} // namespace vlr
