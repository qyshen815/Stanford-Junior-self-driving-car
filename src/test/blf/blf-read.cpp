#include <roadrunner.h>
#include <blf.h>

int main(int argc, char **argv)
{
  blf_t *blf = NULL;
  int ret, cnt = 0;
  blf_pkt_t pkt;

  if(argc != 2) 
    dgc_fatal_error("Usage: %s <FILE>\n", argv[0]);

  blf = new blf_t;
  if(blf->open(argv[1], "r") != BLF_OK) 
    dgc_fatal_error("Can't open BLF file %s for reading.", argv[1]);

  /* ladybug packet are 1MB */
  pkt.max_len = 2000000;
  pkt.data = (unsigned char *)malloc(pkt.max_len);
  dgc_test_alloc(pkt.data);

  while((ret = blf->read_pkt(&pkt)) == BLF_OK) {
    fprintf(stderr, "=> cnt   = %d\n", ++cnt);
    fprintf(stderr, "=> id    = %d\n", pkt.id);
    fprintf(stderr, "=> len   = %d\n", pkt.len);
    fprintf(stderr, "=> time  = %f\n\n", pkt.timestamp);
  }
  blf->close();
  return 0;
}
