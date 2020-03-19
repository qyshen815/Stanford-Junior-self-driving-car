#ifndef DGC_TEXTURES_H
#define DGC_TEXTURES_H

#include <roadrunner.h>
#include <image.h>

namespace vlr {

typedef struct {
  int invert, needs_tex2D;
  int max_texture_size;
  unsigned int texture_id;
  int texture_width, texture_height;
  int image_width, image_height;
  float max_u, max_v;
} dgc_gl_texture_t;

dgc_gl_texture_t* dgc_gl_empty_texture(int width, int height, int max_texture_size, int invert);
dgc_gl_texture_t* dgc_gl_empty_texture_rgba(int width, int height, int max_texture_size, int invert);
dgc_gl_texture_t* dgc_gl_load_texture_from_raw(unsigned char *image, int image_width, int image_height, int max_texture_size, int invert);
dgc_gl_texture_t* dgc_gl_load_texture_from_image(dgc_image_t* image, int max_texture_size, int invert);
dgc_gl_texture_t* dgc_gl_load_texture_from_file(char *filename, int max_texture_size, int invert);
dgc_gl_texture_t* dgc_gl_load_texture_from_bytes(int len, unsigned char *bytes, int max_texture_size, int invert);
dgc_gl_texture_t* dgc_gl_load_texture_rgba_from_raw(unsigned char *image, int image_width, int image_height, int max_texture_size,
    unsigned char invisr, unsigned char invisg, unsigned char invisb, int invert);
dgc_gl_texture_t* dgc_gl_load_texture_rgba_from_image(dgc_image_t* image, int max_texture_size, unsigned char invisr,
    unsigned char invisg, unsigned char invisb, int invert);
dgc_gl_texture_t* dgc_gl_load_texture_rgba_from_file(char *filename, int max_texture_size, unsigned char invisr,
    unsigned char invisg, unsigned char invisb, int invert);
dgc_gl_texture_t* dgc_gl_load_texture_rgba_from_bytes(int len, unsigned char *bytes, int max_texture_size,
    unsigned char invisr, unsigned char invisg, unsigned char invisb, int invert);

void dgc_gl_update_texture_from_raw(dgc_gl_texture_t* texture, unsigned char *image, int image_width, int image_height);
void dgc_gl_update_texture_from_image(dgc_gl_texture_t* texture, dgc_image_t* image);
void dgc_gl_update_texture_from_bytes(dgc_gl_texture_t* texture, int len, unsigned char *bytes);
void dgc_gl_update_texture_rgba_from_raw(dgc_gl_texture_t* texture, unsigned char *image, int image_width, int image_height);
void dgc_gl_free_texture(dgc_gl_texture_t* texture);
void dgc_gl_draw_texture(dgc_gl_texture_t* texture, double x1, double y1, double x2, double y2, int smooth);

} // namespace vlr

#endif
