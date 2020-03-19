#ifndef DGC_B64_H
#define DGC_B64_H

int dgc_b64_encoded_length(int srclen);

int dgc_b64_decoded_length(unsigned char *src, int srclen);

int dgc_b64_encode(unsigned char *src, int srclen, unsigned char *dest, 
                   int destlen);

int dgc_b64_decode(unsigned char *src, int srclen, unsigned char *dest,
                   int destlen);

#endif
