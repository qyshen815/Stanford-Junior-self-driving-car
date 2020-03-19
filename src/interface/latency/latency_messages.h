#ifndef DGC_LATENCY_MESSAGES_H
#define DGC_LATENCY_MESSAGES_H

#include <ipc_interface.h>

namespace dgc {

typedef struct {
  char intercepted;
  double hop_timestamp;
  char hop_host[10];
  double timestamp;
  char host[10];
} LatencyRoundtrip;

#define      DGC_LATENCY_ROUNDTRIP_NAME     "dgc_latency_roundtrip"
#define      DGC_LATENCY_ROUNDTRIP_FMT      "{char,double,[char:10],double,[char:10]}"

const IpcMessageID LatencyRoundtripID = { DGC_LATENCY_ROUNDTRIP_NAME, 
					  DGC_LATENCY_ROUNDTRIP_FMT };

typedef struct {
  double latency;
  double timestamp;
  char host[10];
} LatencyStats;

#define      DGC_LATENCY_STATS_NAME     "dgc_latency_stats"
#define      DGC_LATENCY_STATS_FMT      "{double,double,[char:10]}"

const IpcMessageID LatencyStatsID = { DGC_LATENCY_STATS_NAME, 
				      DGC_LATENCY_STATS_FMT };

}

#endif
