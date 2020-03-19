#include <roadrunner.h>

static char *b64_key =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

int dgc_b64_encoded_length(int srclen)
{
  return ceil(srclen / 3.0) * 4;
}

int dgc_b64_decoded_length(unsigned char *src, int srclen)
{
  int trailing_equals = 0;
  
  if(src[srclen - 1] == '=')
    trailing_equals++;
  if(src[srclen - 2] == '=')
    trailing_equals++;
  return ceil(srclen / 4.0) * 3 - trailing_equals;
}

int dgc_b64_encode(unsigned char *src, int srclen, unsigned char *dest, 
                   int destlen)
{
  unsigned char buffer[10];
  int buffer_pos = 0, i, b1, b2, b3, b4;
  int mark = 0;

  if(destlen < dgc_b64_encoded_length(srclen))
    return -1;
  for(i = 0; i < srclen; i++) {
    buffer[buffer_pos] = src[i];
    buffer_pos++;
    if(buffer_pos == 3) {
      b1 = (buffer[0] >> 2);
      b2 = ((buffer[0] & 3) << 4) | (buffer[1] >> 4);
      b3 = ((buffer[1] & 0x0F) << 2) | (buffer[2] >> 6);
      b4 = (buffer[2] & 0x3F);
      dest[mark++] = b64_key[b1];
      dest[mark++] = b64_key[b2];
      dest[mark++] = b64_key[b3];
      dest[mark++] = b64_key[b4];
      buffer_pos = 0;
    }
  }
  if(buffer_pos != 0) {
    for(i = buffer_pos; i < 3; i++)
      buffer[buffer_pos] = 0;
    b1 = (buffer[0] >> 2);
    b2 = ((buffer[0] & 3) << 4) | (buffer[1] >> 4);
    b3 = ((buffer[1] & 0x0F) << 2) | (buffer[2] >> 6);
    b4 = (buffer[2] & 0x3F);
    dest[mark] = b64_key[b1];
    dest[mark + 1] = b64_key[b2];
    dest[mark + 2] = b64_key[b3];
    dest[mark + 3] = b64_key[b4];
    for(i = buffer_pos; i < 3; i++)
      dest[mark + i + 1] = '=';
  }
  return 0;
}

static inline unsigned char byte_decode(unsigned char c)
{
  if(c >= 'A' && c <= 'Z')
    return c - 'A';
  else if(c >= 'a' && c <= 'z')
    return c - 'a' + 26;
  else if(c >= '0' && c <= '9')
    return c - '0' + 52;
  else if(c == '+')
    return 62;
  else if(c == '/')
    return 63;
  else if(c == '=')
    return 0;
  return 64;
}

int dgc_b64_decode(unsigned char *src, int srclen, unsigned char *dest,
                   int destlen)
{
  unsigned int b1, b2, b3, b4, n;
  int i, mark = 0;

  if(srclen % 4 != 0) 
    return -1;
  if(destlen < dgc_b64_decoded_length(src, srclen))
    return -1;

  for(i = 0; i < srclen; i += 4) {
    b1 = byte_decode(src[i]);
    b2 = byte_decode(src[i + 1]);
    b3 = byte_decode(src[i + 2]);
    b4 = byte_decode(src[i + 3]);
    if(b1 == 64 || b2 == 64 || b3 == 64 || b4 == 64)
      return -1;
    n = (b1 << 18) | (b2 << 12) | (b3 << 6) | (b4);
    dest[mark + 2] = (n & 0xff);
    n = (n >> 8);
    dest[mark + 1] = (n & 0xff);
    n = (n >> 8);
    dest[mark] = (n & 0xff);
    mark += 3;
  }
  return 0;
}

