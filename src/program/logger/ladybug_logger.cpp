#include <roadrunner.h>
#include <blf.h>
#include <blf_id.h>
#include <ladybug_shm_interface.h>

using namespace dgc;

bool quit_signal = false;

void shutdown_module(int sig)
{
  if(sig == SIGINT) 
    quit_signal = true;
}

void print_stats(int duration, long long int bytes, double rate)
{
  fprintf(stderr, "\rLLF-LOGGER : RECORDING [");
  dgc_fprintf_blue(stderr, "%02d:%02d:%02d", 
		   duration / 3600, (duration / 60) % 60, duration % 60);
  fprintf(stderr, "] [");
  dgc_fprintf_blue(stderr, "%5.3f GB", bytes / 1073741824.0);
  fprintf(stderr, "] [");

  dgc_fprintf_blue(stderr, "%3.1f MB", rate);
  fprintf(stderr, "]       ");
}

int main(int argc, char **argv)
{
  double start_time, current_time, last_time = 0;
  LadybugInterface *lbug_interface = NULL;
  int bytes = 0, o_bytes = 0;
  unsigned short id;
  LadybugPacket *pkt;
  blf_t *blf = NULL;
  char *fname;

  if(argc != 2) 
    dgc_fatal_error("Usage: %s <LADYBUG-LOGNAME>\n", argv[0]);

  blf = new blf_t;
  fname = dgc_timestamped_filename(argv[1], ".llf");
  if(blf->open(fname, "w") < 0) 
    dgc_fatal_error("Can't write to file %s.", fname);
  free(fname);

  pkt = new LadybugPacket;

  lbug_interface = new LadybugShmInterface;
  if(lbug_interface->CreateClient() < 0)
    dgc_fatal_error("Could not connect to ladybug interface.");

  signal(SIGINT, shutdown_module);

  start_time = last_time = dgc_get_time();
  while(!quit_signal) {
    while(lbug_interface->DataWaiting()) 
      if(lbug_interface->ReadPacket(pkt) >= 0 && pkt->len > 0) {
	if (pkt->version==LADYBUG_VERSION_2) {
	  id = BLF_LADYBUG2_ID;
	} else if (pkt->version==LADYBUG_VERSION_3) {
	  id = BLF_LADYBUG3_ID;
	} else {
	  id = BLF_UNKNOWN_ID;
	}
	blf->write_data(pkt->len, pkt->data, pkt->timestamp, id);
	bytes += pkt->len;
	o_bytes += pkt->len;
      }

    current_time = dgc_get_time();
    if(current_time - last_time > 0.25) {
      print_stats((int)floor(current_time - start_time), bytes,
		  (o_bytes / (current_time - last_time)) / (1024.0 * 1024.0));
      last_time = current_time;
      o_bytes = 0;
    }
    usleep(10000);
  }

  delete lbug_interface;
  return 0;
}
