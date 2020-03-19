#include <roadrunner.h>
#include <velodyne_shm_interface.h>
#include <async_writer.h>

using namespace dgc;

#define MAX_DATA_LENGTH   (5*1024*1024)

bool quit_flag = false;

void shutdownModule(int sig)
{
  if(sig == SIGINT)
    quit_flag = true;
}

int main(int argc, char **argv)
{
  VelodyneInterface *velo_interface = NULL;
  unsigned char data[MAX_DATA_LENGTH];
  double start_time, current_time;
  int duration, bytes = 0, n;
  dgc::AsyncWriter writer;
  double last_print = 0;
  char *fname;

  if (argc != 2) 
    dgc_die("usage: %s <VELO-LOGNAME>\n\n", argv[0]);

  velo_interface = new VelodyneShmInterface;
  if (velo_interface->CreateClient() < 0)
    dgc_fatal_error("Could not connect to velodyne interface.\n");

  signal(SIGINT, shutdownModule);

  // FIXME (mmde): 64MB write buffer size (1000 * 64K) copied from
  // previous version of velodyne_logger. I think this could be reduced.
  fname = dgc_timestamped_filename(argv[1], ".vlf");
  if (writer.Open(fname, 1000) < 0)
    dgc_fatal_error("Could not open file %s for writing.", fname);
  free(fname);
  
  start_time = dgc_get_time();
  while (!quit_flag) {
    while (velo_interface->RawDataWaiting()) {
      n = velo_interface->ReadRaw(data);
      if (writer.Write(n, data) < 0)
	dgc_warning("Asyncronous writing error.");
      bytes += n;
    }

    current_time = dgc_get_time();
    if (current_time - last_print > 0.5) {
      last_print = current_time;
      duration = (int)floor(current_time - start_time);
      fprintf(stderr, 
	      "\r# INFO: record time: %02d:%02d:%02d (%5.2f MB)    ", 
	      (duration / 3600), (duration / 60) % 60,
	      duration % 60, bytes / 1048576.0);
    }
    usleep(10000);
  }
  writer.Close();
  return 0;
}
