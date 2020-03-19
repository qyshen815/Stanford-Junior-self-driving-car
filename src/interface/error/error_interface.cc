#include <roadrunner.h>
#include <error_messages.h>
#include "logio.h"

namespace dgc {

char *StringToErrorString(char *string, ErrorString *error)
{ 
  static int    num_allocated = 0;
  static char  *line;
  char         *ptr, *running = string;
  char         *t0, *t1 = NULL, *t2 = NULL, *t3 = NULL;
  int           ctr = 0, len;

  len = strlen(string);

  if(num_allocated == 0)
    error->string = NULL;
  
  if(num_allocated < len + 1) {
    line = (char *)realloc(line, (2 * len) * sizeof(char));
    num_allocated = 2 * len;
  }
  strncpy(line, string, len + 1);
  t0 = string;

  running = string; 
  ctr = 0;
  while((ptr = strtok(ctr == 0 ? running : NULL, " ")) != NULL) {
    t1 = t2;
    t2 = t3;
    t3 = ptr;
    ctr++;
  }
  
  if(error->string != NULL)
    free(error->string);
  len = t1 - t0;
  error->string = (char *)malloc((len+1)*sizeof(char));
  strncpy(error->string, line, len);
  error->string[len] = '\0';
  error->timestamp = READ_DOUBLE(&t1);
  READ_HOST(error->host, &t2);
  return string;
}

void ErrorAddLogReaderCallbacks(LogReaderCallbackList *callbacks)
{
  callbacks->AddCallback("ERROR", ErrorStringID,
			 (LogConverterFunc)StringToErrorString, 
			 sizeof(ErrorString), 1);
  callbacks->AddCallback("STATUS", ErrorStatusID, 
			 (LogConverterFunc)StringToErrorString, 
			 sizeof(ErrorString), 1);
}

void ErrorStringWrite(ErrorString *error, double logger_timestamp, 
		      dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "ERROR %s %f %s %f\n", error->string, error->timestamp,
	      error->host, logger_timestamp);
}

void ErrorCommentWrite(ErrorString *error, double logger_timestamp,
		       dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "COMMENT %s %f %s %f\n", error->string, error->timestamp,
             error->host, logger_timestamp);
}

void ErrorStatusWrite(ErrorString *error, double logger_timestamp, 
		      dgc_FILE *outfile)
{
  dgc_fprintf(outfile, "STATUS %s %f %s %f\n", error->string, error->timestamp,
             error->host, logger_timestamp);
}

void ErrorAddLogWriterCallbacks(IpcInterface *ipc, double start_time, 
				dgc_FILE *logfile, 
				dgc_subscribe_t subscribe_how)
{
  ipc->AddLogHandler(ErrorStringID, NULL, sizeof(ErrorString),
		     (dgc_log_handler_t)ErrorStringWrite,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(ErrorCommentID, NULL, sizeof(ErrorString),
		     (dgc_log_handler_t)ErrorCommentWrite,
		     start_time, logfile, subscribe_how);
  ipc->AddLogHandler(ErrorStatusID,  NULL, sizeof(ErrorString),
		     (dgc_log_handler_t)ErrorStatusWrite,
		     start_time, logfile, subscribe_how);
}

void SendErrorString(IpcInterface *ipc, char* fmt, ...) 
{
  static ErrorString msg;
  static int first = 1;
  char string[1000];
  va_list args;
  int err;

  va_start(args, fmt);
  vsprintf(string, fmt, args);
  va_end(args);

  if(first) {
    strcpy(msg.host, dgc_hostname());
    err = ipc->DefineMessage(ErrorStringID);
    TestIpcExit(err, "Could not define message", ErrorStringID);
    first = 0;
  }

  msg.string = string;
  msg.timestamp = dgc_get_time();
  err = ipc->Publish(ErrorStringID, &msg);
  TestIpc(err, "Could not publish", ErrorStringID);
}

void SendErrorComment(IpcInterface *ipc, char* fmt, ...) 
{
  static ErrorString msg;
  static int first = 1;
  char comment[1000];
  va_list args;
  int err;

  va_start(args, fmt);
  vsprintf(comment, fmt, args);
  va_end(args);

  if(first) {
    strcpy(msg.host, dgc_hostname());
    err = ipc->DefineMessage(ErrorCommentID);
    TestIpcExit(err, "Could not define message", ErrorCommentID);
    first = 0;
  }
  msg.string = comment;
  msg.timestamp = dgc_get_time();
  err = ipc->Publish(ErrorCommentID, &msg);
  TestIpc(err, "Could not publish", ErrorCommentID);
}

void SendErrorStatus(IpcInterface *ipc, char* fmt, ...) 
{
  static ErrorString msg;
  static int first = 1;
  char status[1000];
  va_list args;
  int err;

  va_start(args, fmt);
  vsprintf(status, fmt, args);
  va_end(args);

  if(first) {
    strcpy(msg.host, dgc_hostname());
    err = ipc->DefineMessage(ErrorStatusID);
    TestIpcExit(err, "Could not define message", ErrorStatusID);
    first = 0;
  }

  msg.string = status;
  msg.timestamp = dgc_get_time();
  err = ipc->Publish(ErrorStatusID, &msg);
  TestIpc(err, "Could not publish", ErrorStatusID);
}

}
