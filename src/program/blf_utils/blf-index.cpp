#include <roadrunner.h>
#include <blf.h>
#include <blf_id.h>

int main(int argc, char **argv)
{
  int ret, percent_i, last_percent = 0;
  char *index_filename;
  __off64_t offset = 0;
  blf_t *blf = NULL;
  struct stat64 st;
  blf_pkt_t pkt;
  dgc_FILE *fp;

  if(argc != 2) 
    dgc_fatal_error("Usage: %s <LOG-FILE>", argv[0]);

  blf = new blf_t;
  if(blf->open(argv[1], "r") != BLF_OK) 
    dgc_fatal_error("Can't open file %s", argv[1]);

  index_filename = (char *)calloc(strlen(argv[1]) + 10, 1);
  dgc_test_alloc(index_filename);
  strcpy(index_filename, argv[1]);
  strcat(index_filename, ".idx.gz");

  fp = dgc_fopen(index_filename, "w");
  if(fp == NULL)
    dgc_fatal_error("Could not open index file %s for writing.", 
		    index_filename);

  /* ladybug packet are <5MB */
  pkt.max_len = 5000000;
  pkt.data = (unsigned char *)malloc(pkt.max_len);
  dgc_test_alloc(pkt.data);

  stat64(argv[1], &st);
  do {
    offset = blf->tell();
    ret = blf->read_pkt(&pkt);
    if(ret == BLF_OK) 
      dgc_fprintf(fp, "%d %f %lld\n", pkt.id, pkt.timestamp, offset);

    percent_i = (int)rint(100.0 * offset / st.st_size);
    if(percent_i != last_percent) {
      fprintf(stderr, "\r# INFO: indexing: %d%% done", percent_i);
      last_percent = percent_i;
    }
  } while(ret == BLF_OK);
  fprintf(stderr, "\n");

  dgc_fclose(fp);
  blf->close();
  return 0;
}
