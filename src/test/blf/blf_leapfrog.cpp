#include <roadrunner.h>
#include <blf.h>

int main(int argc, char **argv)
{
  blf_t *blf = NULL;
  bool done = false;
  int err;
  unsigned short int pkt_id;
  unsigned int pkt_len;
  double pkt_timestamp, start_time;
  int packet_count = 0;
  if(argc < 2)
    dgc_fatal_error("Usage: %s <BLF-filename>", argv[0]);

  blf = new blf_t;
  if(blf->open(argv[1], "r") != BLF_OK) 
    dgc_fatal_error("Could not open %s for reading.", argv[1]);

  start_time = dgc_get_time();
  do {
    err = blf->start_partial_read(&pkt_id, &pkt_timestamp, &pkt_len);
    if(err != BLF_OK)
      done = true;
    else
      packet_count++;
    
    if(!done) {
      err = blf->abort_partial_read();
      if(err != BLF_OK)
	done = true;
    }
  } while(!done);
  fprintf(stderr, "Time to leapfrog %d messages (%lld bytes) %.3f s\n", 
	  packet_count, blf->tell(), dgc_get_time() - start_time);
  blf->close();
  return 0;
}
