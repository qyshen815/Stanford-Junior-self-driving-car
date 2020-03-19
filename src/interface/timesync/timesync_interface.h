#ifndef DGC_TIMESYNC_UTILS_H
#define DGC_TIMESYNC_UTILS_H

#include <vector>
#include <timesync_messages.h>
#include <logio.h>

namespace dgc {

class TimeSync {
public:
  TimeSync();
  ~TimeSync();

  void updateSync(TimesyncSync *s);
  double getServerTime(void);
  double getServerTime(double time, char *host);

private:
  char   hostname[10];
  double offset;
  double drift;
  double base_time;
  std::vector <TimesyncSync> sync;

};

char *StringToTimesyncSync(char *string, TimesyncSync *sync);
void TimesyncAddLogReaderCallbacks(LogReaderCallbackList *callbacks);
void TimesyncAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				   dgc_FILE *logfile, dgc_subscribe_t how);

}

#endif
