#ifndef DGC_IMAGEIPP_H
#define DGC_IMAGEIPP_H

#include <roadrunner.h>
#include <image.h>

namespace vlr {
void dgc_imageipp_init(void);

void dgc_imageipp_close(void);

dgc_image_t* dgc_imageipp_read(const char* filename);

dgc_image_t* dgc_imageipp_read_from_bytes(int input_buffer_size, const uint8_t* input_buffer);
}  // namespace vlr
#endif
