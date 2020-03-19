#include <roadrunner.h>
#include <blf.h>

int main(int argc, char **argv)
{
  int i, cnt = 0, n, t = 0;
  blf_t *blf = NULL;
  blf_pkt_t pkt;

  if(argc != 2) 
    dgc_fatal_error("Usage: %s <FILE>\n", argv[0]);

  pkt.len  = 100000;
  pkt.data = (unsigned char *)malloc(pkt.len);
  dgc_test_alloc(pkt.data);

  for(i = 0; i < (int)pkt.len; i++) 
    pkt.data[i] = (unsigned char)i;
  pkt.id = 0;

  blf = new blf_t;
  if(blf->open(argv[1], "w") != BLF_OK)
    dgc_fatal_error("Can't open file %s for writing.", argv[1]);

  for(i = 1; i <= 100; i++) {
    pkt.timestamp = dgc_get_time();
    fprintf(stderr, "===> [%d] time   = %f\n", ++cnt, pkt.timestamp);
    n = blf->write_pkt(&pkt);
    t += n;
  }

  usleep(100000);
  blf->close();
  return 0;
}
