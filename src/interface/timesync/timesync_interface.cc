#include <roadrunner.h>
#include <timesync_interface.h>
#include <logio.h>

namespace dgc {

TimeSync::TimeSync(void)
{
  strcpy(hostname, dgc_hostname());
  offset    = 0.0;
  drift     = 0.0;
  base_time = 0.0;
}

TimeSync::~TimeSync(void)
{
}

void 
TimeSync::updateSync(TimesyncSync *s)
{
  unsigned int i;
  if (!strcmp(hostname,s->host)) {
    offset    = s->offset;
    drift     = s->drift;
    base_time = s->base_time;
  } else {
    for(i=0; i<sync.size(); i++) {
      if(!strcmp(s->host, sync[i].host)) {
	sync[i] = *s;
	return;
      }
    }
    // Add new host to vector
    sync.push_back(*s);
  }
}

double 
TimeSync::getServerTime(void)
{
  double t = dgc_get_time();
  return( t + offset + drift * ( t - base_time ) );
}

double 
TimeSync::getServerTime(double t, char *host)
{
  unsigned int i;
  if (!strcmp(hostname,host)) {
    return( t + offset + drift * ( t - base_time ) );
  } else {
    for(i=0; i<sync.size(); i++) {
      if(!strcmp(host, sync[i].host)) {
	return( t + sync[i].offset + 
		sync[i].drift * ( t - sync[i].base_time ) );
      }
    }
    // host not found, return time t
    return t;
  }
}

char *StringToTimesyncSync(char *string, TimesyncSync *sync)
{
  char *pos = string;
  sync->base_time = READ_DOUBLE(&pos);
  sync->offset = READ_DOUBLE(&pos);
  sync->drift = READ_DOUBLE(&pos);
  sync->rms_err = READ_DOUBLE(&pos);
  sync->timestamp = READ_DOUBLE(&pos);
  READ_HOST(sync->host, &pos);
  return pos;
}

void TimesyncAddLogReaderCallbacks(LogReaderCallbackList *callbacks)
{
  callbacks->AddCallback("TIMESYNC", TimesyncSyncID,
			 (LogConverterFunc)StringToTimesyncSync, 
			 sizeof(TimesyncSync), 0);
}

void TimesyncSyncWrite(TimesyncSync *sync,
		       double logger_timestamp, dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "TIMESYNC %.10f %.10f %.10f %.10f %.10f %s %f\n", 
	      sync->base_time, sync->offset,
	      sync->drift, sync->rms_err,
	      sync->timestamp, sync->host, logger_timestamp);
}

void TimesyncAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				   dgc_FILE *logfile, 
				   dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(TimesyncSyncID, NULL, sizeof(TimesyncSync),
		     (dgc_log_handler_t)TimesyncSyncWrite,
		     start_time, logfile, subscribe_how);
}

}
