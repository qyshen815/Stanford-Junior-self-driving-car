#include "global.h"

namespace vlr {

double Time::time_scale_ = 1.0;
double Time::t_=0;
struct timeval Time::tv_;

double Time::current() {

  if (gettimeofday(&tv_, NULL) < 0) {
    dgc_warning("dgc_get_time encountered error in gettimeofday : %s\n", strerror(errno));
  }

  t_ = tv_.tv_sec + tv_.tv_usec / 1000000.0;
  return t_ * time_scale_;
}

} // namespace vlr
