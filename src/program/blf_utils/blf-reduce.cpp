#include <roadrunner.h>
#include <blf.h>

int main(int argc, char **argv)
{
  int ret, cnt = 0, percent_i, last_percent = 0;
  blf_t *blf_i = NULL, *blf_o = NULL;
  double s_time = 0, hz, time_d;
  __off64_t offset = 0;
  int firsttime = 1;
  struct stat64 st;
  blf_pkt_t pkt;

  if(argc != 4) 
    dgc_fatal_error("Usage: %s <HZ> <IN-FILE> <OUT-FILE>", argv[0]);

  hz = atof(argv[1]);
  if(hz <= 0.00001) 
    dgc_fatal_error("Hz must be > 0 (is %.3f)", hz);
  time_d = 1.0 / hz;

  blf_i = new blf_t;
  if(blf_i->open(argv[2], "r") != BLF_OK) 
    dgc_fatal_error("Can't open file %s for reading.", argv[2]);

  blf_o = new blf_t;
  if(blf_o->open(argv[3], "w") != BLF_OK) 
    dgc_fatal_error("Can't open file %s for writing.", argv[3]);

  /* ladybug packet are 1MB */
  pkt.max_len = 2000000;
  pkt.data = (unsigned char *)malloc(pkt.max_len);
  dgc_test_alloc(pkt.data);

  dgc_info("Starting to convert file %s.", argv[2]);

  stat64(argv[2], &st);
  while((ret = blf_i->read_pkt(&pkt)) == BLF_OK) {
    if(firsttime) {
      s_time = pkt.timestamp;
      firsttime = 0;
    }
    if(pkt.timestamp >= s_time + cnt * time_d) {
      blf_o->write_pkt(&pkt);
      cnt++;
    }
    offset = blf_i->tell();
    percent_i = (int)rint(100.0 * offset / st.st_size);
    if(percent_i != last_percent) {
      fprintf(stderr, "\r# INFO: reducing: %d%% done", percent_i);
      last_percent = percent_i;
    }
  }
  fprintf(stderr, "\n");

  usleep(100000);
  blf_i->close();
  blf_o->close();
  return 0;
}
