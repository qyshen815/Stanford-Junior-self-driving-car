#ifndef DGC_TIMESYNC_H
#define DGC_TIMESYNC_H

typedef struct {
  int count;
  double timestamp;
} time_query_t, *time_query_p;

typedef struct {
  int count;
  double query_timestamp;
  double server_timestamp;
} time_response_t, *time_response_p;

#define        SERVER_PORT         5001
#define        CLIENT_PORT         5002

#endif
