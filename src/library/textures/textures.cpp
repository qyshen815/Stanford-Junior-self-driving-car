#include <GL/gl.h>
#include <GL/glut.h>
#include <roadrunner.h>
#include <image.h>
#include "textures.h"

namespace vlr {

void resize_image_data(unsigned char *output, int output_width, int output_height, unsigned char *input,
    int input_width, int input_height, int bytespp, int invert) {
  int x, y, x2, y2;

  if (invert) {
    if (output_width < input_width || output_height < input_height) {
      for (x = 0; x < output_width; x++)
        for (y = 0; y < output_height; y++) {
          x2 = x / (double) output_width * input_width;
          y2 = y / (double) output_height * input_height;
          if (x2 >= input_width) x2 = input_width - 1;
          if (y2 >= input_height) y2 = input_height - 1;
          memcpy(output + (y * output_width + x) * bytespp, input + ((input_height - y2 - 1) * input_width + x2)
              * bytespp, bytespp);
        }
    }
    else {
      for (x = 0; x < input_width; x++)
        for (y = 0; y < input_height; y++)
          memcpy(output + (y * output_width + x) * bytespp, input + ((input_height - y - 1) * input_width + x)
              * bytespp, bytespp);
    }
  }
  else {
    if (output_width < input_width || output_height < input_height) {
      for (x = 0; x < output_width; x++)
        for (y = 0; y < output_height; y++) {
          x2 = x / (double) output_width * input_width;
          y2 = y / (double) output_height * input_height;
          if (x2 >= input_width) x2 = input_width - 1;
          if (y2 >= input_height) y2 = input_height - 1;
          memcpy(output + (y * output_width + x) * bytespp, input + (y2 * input_width + x2) * bytespp, bytespp);
        }
    }
    else {
      for (x = 0; x < input_width; x++)
        for (y = 0; y < input_height; y++)
          memcpy(output + (y * output_width + x) * bytespp, input + (y * input_width + x) * bytespp, bytespp);
    }
  }
}

int update_texture_size(dgc_gl_texture_t* texture, int image_width, int image_height) {
  int prev_texture_width, prev_texture_height;

  prev_texture_width = texture->texture_width;
  prev_texture_height = texture->texture_height;

  texture->image_width = image_width;
  texture->image_height = image_height;

  texture->texture_width = 32;
  texture->texture_height = 32;
  while (texture->texture_width < texture->image_width)
    texture->texture_width *= 2;
  while (texture->texture_height < texture->image_height)
    texture->texture_height *= 2;
  if (texture->texture_height > texture->max_texture_size) texture->texture_height = texture->max_texture_size;
  if (texture->texture_width > texture->max_texture_size) texture->texture_width = texture->max_texture_size;

  texture->max_u = texture->image_width / (double) texture->texture_width;
  texture->max_v = texture->image_height / (double) texture->texture_height;
  if (texture->max_u > 1) texture->max_u = 1;
  if (texture->max_v > 1) texture->max_v = 1;

  return (texture->texture_width != prev_texture_width || texture->texture_height != prev_texture_height);
}

dgc_gl_texture_t* dgc_gl_empty_texture(int width, int height, int max_texture_size, int invert) {
  dgc_gl_texture_t* texture;

  texture = (dgc_gl_texture_t*) calloc(1, sizeof(dgc_gl_texture_t));
  dgc_test_alloc(texture);

  texture->invert = invert;
  texture->max_texture_size = max_texture_size;
  update_texture_size(texture, width, height);
  texture->needs_tex2D = 1;

  glGenTextures(1, &texture->texture_id);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glBindTexture(GL_TEXTURE_2D, texture->texture_id);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  return texture;
}

dgc_gl_texture_t* dgc_gl_empty_texture_rgba(int width, int height, int max_texture_size, int invert) {
  dgc_gl_texture_t* texture;

  texture = (dgc_gl_texture_t*) calloc(1, sizeof(dgc_gl_texture_t));
  dgc_test_alloc(texture);

  texture->invert = invert;
  texture->max_texture_size = max_texture_size;
  update_texture_size(texture, width, height);
  texture->needs_tex2D = 1;

  glGenTextures(1, &texture->texture_id);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glBindTexture(GL_TEXTURE_2D, texture->texture_id);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  return texture;
}

dgc_gl_texture_t* dgc_gl_load_texture_from_raw(unsigned char *image, int image_width, int image_height,
    int max_texture_size, int invert) {
  dgc_gl_texture_t* texture = dgc_gl_empty_texture(image_width, image_height, max_texture_size, invert);
  dgc_gl_update_texture_from_raw(texture, image, image_width, image_height);
  return texture;
}

dgc_gl_texture_t* dgc_gl_load_texture_from_image(dgc_image_t* image, int max_texture_size, int invert) {
  return dgc_gl_load_texture_from_raw(image->pix, image->width, image->height, max_texture_size, invert);
}

dgc_gl_texture_t* dgc_gl_load_texture_from_file(char *filename, int max_texture_size, int invert) {
  dgc_gl_texture_t* texture;
  dgc_image_t* image;

  /* read the image from file */
  image = dgc_image_read(filename);
  if (image == NULL) {
    fprintf(stderr, "Error: could not read image %s\n", filename);
    return NULL;
  }

  texture = dgc_gl_load_texture_from_raw(image->pix, image->width, image->height, max_texture_size, invert);
  dgc_image_free(image);
  return texture;
}

dgc_gl_texture_t* dgc_gl_load_texture_from_bytes(int len, unsigned char *bytes, int max_texture_size, int invert) {
  dgc_gl_texture_t* texture;
  dgc_image_t* image;

  /* read the image from memory */
  image = dgc_image_read_from_bytes(len, bytes);
  if (image == NULL) {
    fprintf(stderr, "Error: could not read image from bytes\n");
    return NULL;
  }

  texture = dgc_gl_load_texture_from_raw(image->pix, image->width, image->height, max_texture_size, invert);
  dgc_image_free(image);
  return texture;
}

dgc_gl_texture_t* dgc_gl_load_texture_rgba_from_raw(unsigned char *image, int image_width, int image_height,
    int max_texture_size, unsigned char invisr, unsigned char invisg, unsigned char invisb, int invert) {
  dgc_gl_texture_t* texture;
  unsigned char *rgba;
  int r, c;

  texture = dgc_gl_empty_texture_rgba(image_width, image_height, max_texture_size, invert);

  rgba = (unsigned char *) calloc(1, image_width * image_height * 4);

  for (r = 0; r < image_height; r++)
    for (c = 0; c < image_width; c++) {
      if (image[(r * image_width + c) * 3 + 0] != invisr || image[(r * image_width + c) * 3 + 1] != invisg || image[(r
          * image_width + c) * 3 + 2] != invisb) {
        memcpy(rgba + (r * image_width + c) * 4, image + (r * image_width + c) * 3, 3);
        rgba[(r * image_width + c) * 4 + 3] = 255;
      }
      else
        memset(rgba + (r * image_width + c) * 4, 0, 4);
    }
  dgc_gl_update_texture_rgba_from_raw(texture, rgba, image_width, image_height);
  free(rgba);
  return texture;
}

dgc_gl_texture_t* dgc_gl_load_texture_rgba_from_image(dgc_image_t* image, int max_texture_size, unsigned char invisr,
    unsigned char invisg, unsigned char invisb, int invert) {
  return dgc_gl_load_texture_rgba_from_raw(image->pix, image->width, image->height, max_texture_size, invisr, invisg,
      invisb, invert);
}

dgc_gl_texture_t* dgc_gl_load_texture_rgba_from_file(char *filename, int max_texture_size, unsigned char invisr,
    unsigned char invisg, unsigned char invisb, int invert) {
  dgc_gl_texture_t* texture;
  dgc_image_t* image;

  /* read the image from file */
  image = dgc_image_read(filename);
  if (image == NULL) {
    fprintf(stderr, "Error: could not read image %s\n", filename);
    return NULL;
  }

  texture = dgc_gl_load_texture_rgba_from_raw(image->pix, image->width, image->height, max_texture_size, invisr,
      invisg, invisb, invert);
  dgc_image_free(image);
  return texture;
}

dgc_gl_texture_t* dgc_gl_load_texture_rgba_from_bytes(int len, unsigned char *bytes, int max_texture_size,
    unsigned char invisr, unsigned char invisg, unsigned char invisb, int invert) {
  dgc_gl_texture_t* texture;
  dgc_image_t* image;

  /* read the image from memory */
  image = dgc_image_read_from_bytes(len, bytes);
  if (image == NULL) {
    fprintf(stderr, "Error: could not read image from bytes\n");
    return NULL;
  }
  texture = dgc_gl_load_texture_rgba_from_raw(image->pix, image->width, image->height, max_texture_size, invisr,
      invisg, invisb, invert);
  dgc_image_free(image);
  return texture;
}

void dgc_gl_free_texture(dgc_gl_texture_t* texture) {
  glDeleteTextures(1, &texture->texture_id);
  free(texture);
}

void dgc_gl_update_texture_from_raw(dgc_gl_texture_t* texture, unsigned char *image, int image_width, int image_height) {
  unsigned char *texture_data;

  /* resize the texture, if necessary */
  if (image_width != texture->image_width || image_height != texture->image_height) {
    update_texture_size(texture, image_width, image_height);
    texture->needs_tex2D = 1;
  }

  glBindTexture(GL_TEXTURE_2D, texture->texture_id);

  if (texture->needs_tex2D) {
    texture_data = (unsigned char *) calloc(1, 3 * texture->texture_width * texture->texture_height);
    dgc_test_alloc(texture_data);
    resize_image_data(texture_data, texture->texture_width, texture->texture_height, image, image_width, image_height,
        3, texture->invert);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, texture->texture_width, texture->texture_height, 0, GL_BGR, GL_UNSIGNED_BYTE,
        texture_data);
    texture->needs_tex2D = 0;
    free(texture_data);
  }
  else if (!texture->invert && texture->image_width <= texture->texture_width && texture->image_height
      <= texture->texture_height)
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, texture->image_width, texture->image_height, GL_BGR, GL_UNSIGNED_BYTE,
        image);
  else {
    texture_data = (unsigned char *) calloc(1, 3 * texture->texture_width * texture->texture_height);
    dgc_test_alloc(texture_data);
    resize_image_data(texture_data, texture->texture_width, texture->texture_height, image, image_width, image_height,
        3, texture->invert);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, texture->texture_width, texture->texture_height, GL_BGR, GL_UNSIGNED_BYTE,
        texture_data);
    free(texture_data);
  }
}

void dgc_gl_update_texture_from_image(dgc_gl_texture_t* texture, dgc_image_t* image) {
  dgc_gl_update_texture_from_raw(texture, image->pix, image->width, image->height);
}

void dgc_gl_update_texture_from_bytes(dgc_gl_texture_t* texture, int len, unsigned char *bytes) {
  dgc_image_t* image;

  /* read the image from memory */
  image = dgc_image_read_from_bytes(len, bytes);
  if (image == NULL) {
    fprintf(stderr, "Error: could not read image from bytes\n");
    return;
  }
  dgc_gl_update_texture_from_raw(texture, image->pix, image->width, image->height);
  dgc_image_free(image);
}

void dgc_gl_update_texture_rgba_from_raw(dgc_gl_texture_t* texture, unsigned char *image, int image_width,
    int image_height) {
  unsigned char *texture_data;

  /* resize the texture, if necessary */
  if (image_width != texture->image_width || image_height != texture->image_height) {
    update_texture_size(texture, image_width, image_height);
    texture->needs_tex2D = 1;
  }

  glBindTexture(GL_TEXTURE_2D, texture->texture_id);

  if (texture->needs_tex2D) {
    texture_data = (unsigned char *) calloc(1, 4 * texture->texture_width * texture->texture_height);
    dgc_test_alloc(texture_data);
    resize_image_data(texture_data, texture->texture_width, texture->texture_height, image, image_width, image_height,
        4, texture->invert);
    glTexImage2D(GL_TEXTURE_2D, 0, 4, texture->texture_width, texture->texture_height, 0, GL_RGBA, GL_UNSIGNED_BYTE,
        texture_data);
    free(texture_data);
    texture->needs_tex2D = 0;
  }
  else if (!texture->invert && texture->image_width <= texture->texture_width && texture->image_height
      <= texture->texture_height)
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, texture->image_width, texture->image_height, GL_RGBA, GL_UNSIGNED_BYTE,
        image);
  else {
    texture_data = (unsigned char *) calloc(1, 4 * texture->texture_width * texture->texture_height);
    dgc_test_alloc(texture_data);
    resize_image_data(texture_data, texture->texture_width, texture->texture_height, image, image_width, image_height,
        4, texture->invert);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, texture->texture_width, texture->texture_height, GL_RGBA, GL_UNSIGNED_BYTE,
        texture_data);
    free(texture_data);
  }
}

void dgc_gl_draw_texture(dgc_gl_texture_t* texture, double x1, double y1, double x2, double y2, int smooth) {
  double dx, dy;

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, texture->texture_id);

  if (smooth) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    dx = 1 / (2 * texture->texture_width);
    dy = 1 / (2 * texture->texture_height);
  }
  else {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    dx = 0;
    dy = 0;
  }

  glColor3f(1, 1, 1);
  glBegin(GL_POLYGON);
  glTexCoord2f(dx, dy);
  glVertex2f(x1, y1);
  glTexCoord2f(texture->max_u - dx, dy);
  glVertex2f(x2, y1);
  glTexCoord2f(texture->max_u - dx, texture->max_v - dy);
  glVertex2f(x2, y2);
  glTexCoord2f(dx, texture->max_v - dy);
  glVertex2f(x1, y2);
  glEnd();
  glDisable(GL_TEXTURE_2D);
}

} // namespace vlr
