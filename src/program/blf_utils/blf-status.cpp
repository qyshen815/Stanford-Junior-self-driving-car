#include <roadrunner.h>
#include <blf.h>

int main(int argc, char **argv)
{
  int ret, ts_errors = 0, cs_errors = 0, percent_i, last_percent = 0;
  blf_t *blf_i = NULL;
  struct stat64 st;
  __off64_t offset;
  blf_pkt_t pkt;

  if(argc != 2) 
    dgc_fatal_error("usage: %s <LOG-FILE>", argv[0]);

  blf_i = new blf_t;
  if(blf_i->open(argv[1], "r") != BLF_OK) 
    dgc_fatal_error("Could not open file %s for reading.", argv[1]);

  /* ladybug packet are 1MB */
  pkt.max_len = 2000000;
  pkt.data = (unsigned char *)malloc(pkt.max_len);
  dgc_test_alloc(pkt.data);

  stat64(argv[1], &st);
  do {
    ret = blf_i->read_pkt(&pkt);
    if(ret == BLF_TIMESTAMP)
      ts_errors++;
    else if(ret == BLF_CHKSUM)
      cs_errors++;

    offset = blf_i->tell();
    percent_i = (int)rint(100.0 * offset / st.st_size);
    if(percent_i != last_percent) {
      fprintf(stderr, "\r# INFO: reducing: %d%% done", percent_i);
      last_percent = percent_i;
    }
  } while(ret == BLF_OK);
  fprintf(stderr, "\nEnd of file at byte %lld.\n", 
      (long long int)offset);
  blf_i->close();
  fprintf(stderr, "%d broken timestamp erorrs.\n", ts_errors);
  fprintf(stderr, "%d bad checksum errors.\n", cs_errors);
  return 0;
}
