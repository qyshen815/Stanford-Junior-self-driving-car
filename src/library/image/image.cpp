#include <iostream>
#include <roadrunner.h>
#include <IL/il.h>
#include <highgui.h>

#include "image.h"

namespace vlr {

static bool initialized = false;
static uint32_t img_id = 0;

static dgc_image_t* dgc_image_from_il();
static dgc_image_t* dgc_image_from_mat(cv::Mat& img);

inline void dgc_image_check_initialization()
{
  if(!initialized) {
    ilInit();
    ilGenImages(1, &img_id);
    ilBindImage(img_id);
    initialized = true;
  }
}

dgc_image_t* dgc_image_initialize(int width, int height)
{
  dgc_image_t* temp;

  temp = (dgc_image_t*)calloc(1, sizeof(dgc_image_t));
  dgc_test_alloc(temp);
  temp->height = height;
  temp->width = width;
  temp->pix = (uint8_t *)calloc(width * height * 3, 1);
  dgc_test_alloc(temp->pix);
  return temp;
}

void dgc_image_free(dgc_image_t* image)
{
  free(image->pix);
  free(image);
}

dgc_image_t* dgc_image_read(const char* filename) {

  dgc_image_check_initialization();

  cv::Mat img = cv::imread(filename, 1);
  return dgc_image_from_mat(img);
}

dgc_image_t* dgc_image_from_il() {

  ilConvertImage(IL_RGB, IL_UNSIGNED_BYTE);

  int width = ilGetInteger(IL_IMAGE_WIDTH);
  int height = ilGetInteger(IL_IMAGE_HEIGHT);
  int channels=3;

  uint8_t* img_data = (uint8_t*) ilGetData();
  dgc_image_t* image = dgc_image_initialize(width, height);
  uint8_t* data = (uint8_t*) image->pix;
  if (channels == 1) { //greyscale image
    int ih3 = image->height * 3 - 3;
    for (int j=height-1; j>=0; j--) {
      int h3 = j * 3;
      for (int i = 0; i < width; i++) {
        int i3 = i * 3;
        data[width * ((ih3) - h3) + i3] = img_data[j*width+i];
        data[width * ((ih3) - h3) + i3 + 1] = img_data[j*width+i];
        data[width * ((ih3) - h3) + i3 + 2] = img_data[j*width+i];
      }
    }
  }
  else { //color image
    memcpy(data, img_data, width*height*3*sizeof(uint8_t));
  }
  return image;
}

dgc_image_t* dgc_image_from_mat(cv::Mat& img) {

  int width = img.cols;
  int height = img.rows;
  int channels=3;

  uint8_t* img_data = (uint8_t*) img.data;
  dgc_image_t* image = dgc_image_initialize(width, height);
  uint8_t* data = (uint8_t*) image->pix;
  if (channels == 1) { //greyscale image
    int ih3 = image->height * 3 - 3;
    for (int j=height-1; j>=0; j--) {
      int h3 = j * 3;
      for (int i = 0; i < width; i++) {
        int i3 = i * 3;
        data[width * ((ih3) - h3) + i3 + 2] = img_data[j*width+i];
        data[width * ((ih3) - h3) + i3 + 1] = img_data[j*width+i];
        data[width * ((ih3) - h3) + i3 + 0] = img_data[j*width+i];
      }
    }
  }
  else { //color image
    memcpy(data, img_data, width*height*3*sizeof(uint8_t));
  }
  return image;
}

bool dgc_image_write_raw(uint8_t* image_data, int width, int height, const char* filename) {
  if(!image_data || !filename) {return false;}
  if(width<=0 || height <=0) {return false;}

  dgc_image_check_initialization();

  ilTexImage((ILuint)width, (ILuint)height, 0, 3, IL_RGB, IL_UNSIGNED_BYTE, image_data);
  ilSaveImage(filename);

  int err = ilGetError();
  if(err != IL_NO_ERROR) {
    std::cout << "Could not write image data to file " << filename << "(error code " << err << ")\n";
  return false;
  }
  return true;
}

bool dgc_image_write(dgc_image_t* image, const char* filename) {
  return dgc_image_write_raw(image->pix, image->width, image->height, filename);
}

bool dgc_image_rgba_write_raw(uint8_t* image_data, int width, int height, const char* filename) {
  if(!image_data || !filename) {return false;}
  if(width<=0 || height <=0) {return false;}

  dgc_image_check_initialization();

  ilTexImage((ILuint)width, (ILuint)height, 0, 3, IL_RGBA, IL_UNSIGNED_BYTE, image_data);
  ilSaveImage(filename);

  int err = ilGetError();
  if(err != IL_NO_ERROR) {
    std::cout << "Could not write image data to file " << filename << "(error code " << err << ")\n";
    return false;
  }
  return true;
}

void dgc_image_resize(dgc_image_t* image_in, int width, int height)
{
  int r, c, r2, c2;
  int mark_in, mark_out;
  uint8_t *temp_pix;

  temp_pix = (uint8_t *)calloc(width * height * 3, 1);
  dgc_test_alloc(temp_pix);

  for(r = 0; r < height; r++)
    for(c = 0; c < width; c++) {
      r2 = r / (float)height * image_in->height;
      if(r2 >= image_in->height) r2 = image_in->height - 1;
      c2 = c / (float)width * image_in->width;
      if(c2 >= image_in->width)  c2 = image_in->width - 1;
      mark_in = (r2 * image_in->width + c2) * 3;
      mark_out = (r * width + c) * 3;
      temp_pix[mark_out] = image_in->pix[mark_in];
      temp_pix[mark_out + 1] = image_in->pix[mark_in + 1];
      temp_pix[mark_out + 2] = image_in->pix[mark_in + 2];
    }
  free(image_in->pix);
  image_in->pix = temp_pix;
  image_in->width = width;
  image_in->height = height;
}

void dgc_image_copy(dgc_image_t* image_in, dgc_image_t* image_out)
{
  if(image_in->height != image_out->height ||
     image_in->width != image_out->width)
    dgc_die("Error: cannot copy images of different sizes.\n");
  memcpy(image_out->pix, image_in->pix, 
         image_in->width * image_in->height * 3);
}

inline static int has_jpg_header(const uint8_t* buffer, int buffer_length)
{
  if(buffer_length > 4 && buffer[0] == 0xFF && buffer[1] == 0xD8 &&
     buffer[2] == 0xFF && buffer[3] == 0xE0)
    return 1;
  return 0;
}

dgc_image_t* dgc_image_read_from_bytes(int numbytes, const uint8_t* bytes)
{
  dgc_image_check_initialization();
 if(ilLoadL(IL_TYPE_UNKNOWN, (void*)bytes, (ILuint)numbytes) == IL_FALSE) {
   return NULL;
 }
 return dgc_image_from_il();
}

} // namespace vlr
