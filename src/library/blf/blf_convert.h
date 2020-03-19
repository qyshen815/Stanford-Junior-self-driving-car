#ifndef DGC_BLF_CONVERT_H
#define DGC_BLF_CONVERT_H

u_int16_t
blf_uchar2ushort(unsigned char *bytes);

void
blf_ushort2uchar(u_int16_t i, unsigned char *bytes);

u_int32_t
blf_uchar2uint(unsigned char *bytes);

void
blf_uint2uchar(u_int32_t i, unsigned char *bytes);

u_int64_t
blf_uchar2ulong(unsigned char * bytes);

void
blf_ulong2uchar(u_int64_t l, unsigned char *bytes);

double
blf_uchar2double(unsigned char *bytes);

void
blf_double2uchar(double d, unsigned char *bytes);

unsigned short
twoByteLoHiShort(unsigned char a, unsigned char b);

unsigned short
twoByteHiLoShort(unsigned char a, unsigned char b);

short
twoByteLoHiSignedShort(unsigned char a, unsigned char b);

short
twoByteHiLoSignedShort(unsigned char a, unsigned char b);

unsigned int
convertBytes2UInt8(unsigned char *bytes);

int
convertBytes2Int16(unsigned char *bytes);

#endif
