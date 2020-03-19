#include <roadrunner.h>

u_int16_t
blf_uchar2ushort(unsigned char * bytes)
{
  u_int16_t i;

  i = 256 * bytes[1] + bytes[0];
  return i;
}

void
blf_ushort2uchar(u_int16_t i, unsigned char * bytes)
{
  memcpy(bytes, &i, 2);
}

u_int32_t
blf_uchar2uint(unsigned char *bytes)
{
  u_int32_t i;
  memcpy(&i, bytes, 4);
  return i;
}

void
blf_uint2uchar(u_int32_t i, unsigned char * bytes)
{
  memcpy(bytes, &i, 4);
}

u_int64_t
blf_uchar2ulong(unsigned char * bytes)
{
  u_int64_t l;
  memcpy(&l, bytes, 8);
  return l;
}

void
blf_ulong2uchar(u_int64_t l, unsigned char * bytes)
{
  memcpy(bytes, &l, 8);
}


double
blf_uchar2double(unsigned char *bytes)
{
  double v;
  memcpy(&v, bytes, 8);
  return v;
}

void
blf_double2uchar(double d, unsigned char *bytes)
{
  memcpy(bytes, &d, 8);
}

unsigned short
twoByteLoHiShort(unsigned char a, unsigned char b)
{
  unsigned int r = (unsigned short)b * 256 + (unsigned short)a;
  return r;
}

unsigned short
twoByteHiLoShort(unsigned char a, unsigned char b)
{
  unsigned int r = (unsigned short)a * 256 + (unsigned short)b;
  return r;
}

short
twoByteLoHiSignedShort(unsigned char a, unsigned char b)
{
  unsigned int r = (unsigned short)b * 256 + (unsigned short)a;
  return r;
}

short
twoByteHiLoSignedShort(unsigned char a, unsigned char b)
{
  unsigned int r = (unsigned short)a * 256 + (unsigned short)b;
  return r;
}

unsigned int
convertBytes2UInt8(unsigned char *bytes)
{
  unsigned int i;
  memcpy(&i, bytes, 1);
  return i;
}

int
convertBytes2Int16(unsigned char *bytes)
{
  int i;
  memcpy(&i, bytes, 2);
  return i;
}

unsigned int
convertBytes2UInt16(unsigned char *bytes)
{
  unsigned int i;
  memcpy(&i, bytes, 2);
  return i;
}

unsigned int
convertRevBytes2UInt16(unsigned char *bytes)
{
  unsigned int i;
  unsigned char b[2];
  b[1] = *bytes; b[0] = *(bytes + 1);
  memcpy(&i, b, 2);
  return i;
}

