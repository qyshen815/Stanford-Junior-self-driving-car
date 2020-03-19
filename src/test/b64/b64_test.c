#include <roadrunner.h>
#include <b64.h>

int main(void)
{
  char *src = "Man is distinguished, not only by his reason, but by this singular passion from other animals, which is a lust of the mind, that by a perseverance of delight in the continued and indefatigable generation of knowledge, exceeds the short vehemence of any carnal pleasure.";
  char *dest = NULL;
  int destlen;
  int i;
  char *dest2 = NULL;
  int dest2len = 0;

  /* make a buffer than can accomodate the encoded data */
  destlen = dgc_b64_encoded_length(strlen(src));
  dest = (char *)calloc(destlen + 1, 1);
  dgc_test_alloc(dest);

  /* encode the string */
  dgc_b64_encode(src, strlen(src), dest, destlen);
  dest[destlen] = '\0';
  
  /* print out the encoded data */
  for(i = 0; i < (int)strlen(dest); i++) {
    fprintf(stderr, "%c", dest[i]);
    if(((i + 1) % 80) == 0)
      fprintf(stderr, "\n");
  }
  fprintf(stderr, "\n");
  
  /* make a buffer that can accomodate the decoded data */
  dest2len = dgc_b64_decoded_length(destlen);
  dest2 = (char *)calloc(dest2len + 1, 1);
  dgc_test_alloc(dest2);

  /* decode the data */
  dgc_b64_decode(dest, destlen, dest2, dest2len);
  dest2[dest2len] = '\0';

  /* print out the decoded data */
  fprintf(stderr, "\n");
  for(i = 0; i < (int)strlen(dest2); i++) 
    fprintf(stderr, "%c", dest2[i]);
  fprintf(stderr, "\n");

  return 0;
}

