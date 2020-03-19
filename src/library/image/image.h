#ifndef VLR_IMAGE_H
#define VLR_IMAGE_H
#include <inttypes.h>

#include <stdint.h>

namespace vlr {

typedef struct {
  uint8_t* pix;
  int width, height;
  int nchannels;  
} dgc_image_t;  //  might be renamed/replaced sometime

dgc_image_t* dgc_image_initialize(int width, int height);
void dgc_image_free(dgc_image_t* image);
dgc_image_t* dgc_image_read(const char* filename);
dgc_image_t* dgc_image_read_from_bytes(int numbytes, const uint8_t* bytes);

bool dgc_image_write(dgc_image_t* image, const char* filename);
bool dgc_image_write_raw(uint8_t* image_data, int width, int height, const char* filename);
bool dgc_image_rgba_write_raw(uint8_t* image_data, int width, int height, const char* filename);

void dgc_image_resize(dgc_image_t* image_in, int width, int height);
void dgc_image_copy(dgc_image_t* image_in, dgc_image_t* image_out);
} // namespace vlr

#endif
