#include <sys/time.h>
#include "aw_timestamp.h"

namespace vlr
{

kogmo_timestamp_t
kogmo_timestamp_now (void)
{
  //TODO: this should come from the recording instead of system time
  struct timeval tv;

  if (gettimeofday(&tv, 0) < 0)
    return 0;

  kogmo_timestamp_t t = (int64_t) tv.tv_sec * (int64_t) KOGMO_TIMESTAMP_TICKSPERSECOND + ((int64_t) tv.tv_usec) * ((int64_t) KOGMO_TIMESTAMP_TICKSPERSECOND / (int64_t) 1000000);
  return t;
};
};
