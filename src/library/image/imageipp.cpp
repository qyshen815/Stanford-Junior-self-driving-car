#include <roadrunner.h>
#include <jinclude.ipp.h>
#include <jpeglib.ipp.h>
#include <jerror.ipp.h> 
#include <ctype.h>
#include <ippcore.h>
#include <setjmp.h>
#include "imageipp.h"

namespace vlr {

typedef struct my_error_mgr {
  struct jpeg_error_mgr pub;    /* "public" fields */
  jmp_buf setjmp_buffer;        /* for return to caller */
} my_error_mgr_t;

typedef struct my_error_mgr *my_error_ptr;

static struct my_error_mgr jerr;

static struct jpeg_decompress_struct *cinfo = NULL;

void dgc_image_jpeg_error_exit(j_common_ptr cinfo)
{
  /* cinfo->err really points to a my_error_mgr struct, so coerce pointer */
  my_error_ptr myerr = (my_error_ptr)cinfo->err;

  /* Always display the message. */
  /* We could postpone this until after returning, if we chose. */
  fprintf(stderr, "jpeg error:");
  (*cinfo->err->output_message)(cinfo);

  /* Return control to the setjmp point */
  longjmp(myerr->setjmp_buffer, 1);
}

void dgc_imageipp_init(void) 
{
  cinfo = (jpeg_decompress_struct*)calloc(1, sizeof(struct jpeg_decompress_struct));
  dgc_test_alloc(cinfo);

  if(ippStsNoErr > ippStaticInit())
    dgc_die("Can't initialize IPP library\n");

  /* Initialize the JPEG decompression object with default error handling. */
  cinfo->err = jpeg_std_error((struct jpeg_error_mgr *)&jerr);
  cinfo->err->error_exit = dgc_image_jpeg_error_exit;
  jpeg_create_decompress(cinfo);
}

void dgc_imageipp_close()
{
  jpeg_destroy_decompress(cinfo);
  cinfo = NULL;
}

dgc_image_t* dgc_imageipp_read(const char* filename)
{
  int width, height;
  uint8_t *data;
  JSAMPARRAY buffer = 0;
  dgc_image_t* image = NULL;
  FILE *fp;
  int i;
  fp = fopen(filename, "r");
  if(fp == NULL)
    return NULL;
  
  /* Specify data source for decompression */
  jpeg_stdio_src(cinfo, fp);
  
  /* Read file header, set default decompression parameters */
  jpeg_read_header(cinfo, TRUE);
  
  /* Start decompressor */
  jpeg_start_decompress(cinfo);
  height = cinfo->image_height;
  width = cinfo->image_width;

  image = dgc_image_initialize(width, height);
  data = (uint8_t *)image->pix;
  if (cinfo->num_components == 1) { //greyscale image
    int ih3 = image->height*3-3;
    uint8_t *temp_data = (uint8_t*)malloc(width*sizeof(uint8_t)*3);
    for(; height--;) {
      int h3 = height*3;
      buffer = &temp_data;
      jpeg_read_scanlines(cinfo, buffer, 1);
      for (i = 0; i < width;i++) {
        int i3 = i*3;
        data[width*((ih3)-h3)+i3] = temp_data[i];
        data[width*((ih3)-h3)+i3+1] = temp_data[i];
        data[width*((ih3)-h3)+i3+2] = temp_data[i];
      }
    }
    free(temp_data);
  } else { //color image
    for(; height--;) {
      buffer = &data;
      jpeg_read_scanlines(cinfo, buffer, 1);
      data += width * 3;
    }
  }
  jpeg_finish_decompress(cinfo);
  fclose(fp);
  return image;
}

/* Expanded data source object for memory input */

typedef struct {
  struct jpeg_source_mgr pub;	/* public fields */
  JOCTET eoi_buffer[2];		/* a place to put a dummy EOI */
} my_source_mgr;

typedef my_source_mgr *my_src_ptr;

/*
 * Initialize source --- called by jpeg_read_header
 * before any data is actually read.
 */

METHODDEF(void) init_source(__attribute__ ((unused)) j_decompress_ptr cinfo)
{
  /* No work, since jpeg_memory_src set up the buffer pointer and count.
   * Indeed, if we want to read multiple JPEG images from one buffer,
   * this *must* not do anything to the pointer.
   */
}

/*
 * Fill the input buffer --- called whenever buffer is emptied.
 *
 * In this application, this routine should never be called; if it is called,
 * the decompressor has overrun the end of the input buffer, implying we
 * supplied an incomplete or corrupt JPEG datastream.  A simple error exit
 * might be the most appropriate response.
 *
 * But what we choose to do in this code is to supply dummy EOI markers
 * in order to force the decompressor to finish processing and supply
 * some sort of output image, no matter how corrupted.
 */

METHODDEF(boolean) fill_input_buffer(__attribute__ ((unused))
				     j_decompress_ptr cinfo)
{
  return TRUE;
}

/*
 * Skip data --- used to skip over a potentially large amount of
 * uninteresting data (such as an APPn marker).
 *
 * If we overrun the end of the buffer, we let fill_input_buffer deal with
 * it.  An extremely large skip could cause some time-wasting here, but
 * it really isn't supposed to happen ... and the decompressor will never
 * skip more than 64K anyway.
 */

METHODDEF(void) skip_input_data(j_decompress_ptr cinfo, long num_bytes)
{
  my_src_ptr src = (my_src_ptr)cinfo->src;

  if(num_bytes > 0) {
    while(num_bytes > (long)src->pub.bytes_in_buffer) {
      num_bytes -= (long)src->pub.bytes_in_buffer;
      (void)fill_input_buffer(cinfo);
      /* note we assume that fill_input_buffer will never return FALSE,
       * so suspension need not be handled.
       */
    }
    src->pub.next_input_byte += (size_t) num_bytes;
    src->pub.bytes_in_buffer -= (size_t) num_bytes;
  }
}

/*
 * Terminate source --- called by jpeg_finish_decompress
 * after all data has been read.  Often a no-op.
 *
 * NB: *not* called by jpeg_abort or jpeg_destroy; surrounding
 * application must deal with any cleanup that should happen even
 * for error exit.
 */

METHODDEF(void) term_source(__attribute__ ((unused)) j_decompress_ptr cinfo)
{
  /* no work necessary here */
}

/*
 * Prepare for input from a memory buffer.
 */

GLOBAL(void) jpeg_memory_src(j_decompress_ptr cinfo, const JOCTET *buffer, 
			     size_t bufsize)
{
  my_src_ptr src;

  /* The source object is made permanent so that a series of JPEG images
   * can be read from a single buffer by calling jpeg_memory_src
   * only before the first one.
   * This makes it unsafe to use this manager and a different source
   * manager serially with the same JPEG object.  Caveat programmer.
   */
  if(cinfo->src == NULL) {	/* first time for this JPEG object? */
    cinfo->src = (struct jpeg_source_mgr *)
      (*cinfo->mem->alloc_small)((j_common_ptr) cinfo, JPOOL_PERMANENT,
				 SIZEOF(my_source_mgr));
  }

  src = (my_src_ptr) cinfo->src;
  src->pub.init_source = init_source;
  src->pub.fill_input_buffer = fill_input_buffer;
  src->pub.skip_input_data = skip_input_data;
  src->pub.resync_to_restart = jpeg_resync_to_restart; /* use default method */
  src->pub.term_source = term_source;

  src->pub.next_input_byte = buffer;
  src->pub.bytes_in_buffer = bufsize;
}

dgc_image_t* dgc_imageipp_read_from_bytes(int input_buffer_size, const uint8_t* input_buffer)
{
  int width, height;
  uint8_t *data;
  JSAMPARRAY buffer = 0;
  dgc_image_t* image = NULL;
  int i;
  
  /* Specify data source for decompression */
  jpeg_memory_src(cinfo, input_buffer, input_buffer_size);
  
  /* Read file header, set default decompression parameters */
  jpeg_read_header(cinfo, TRUE);
  
  /* Start decompressor */
  jpeg_start_decompress(cinfo);
  height = cinfo->image_height;
  width = cinfo->image_width;
  
  image = dgc_image_initialize(width, height);
  image->width = width;
  image->height = height;
  image->nchannels = cinfo->num_components;
  data = (uint8_t *)image->pix;
  if (cinfo->num_components == 1) { //greyscale image
    int ih3 = image->height*3-3;
    uint8_t* temp_data = (uint8_t*)malloc(width*sizeof(uint8_t)*3);
    for(; height--;) {
      int h3 = height*3;
      buffer = &temp_data;
      jpeg_read_scanlines(cinfo, buffer, 1);
      for (i = 0; i < width;i++) {
        int i3 = i*3;
        data[width*((ih3)-h3)+i3] = temp_data[i];
        data[width*((ih3)-h3)+i3+1] = temp_data[i];
        data[width*((ih3)-h3)+i3+2] = temp_data[i];
      }
    }
  } else { //color image
    for(; height--;) {
      buffer = &data;
      jpeg_read_scanlines(cinfo, buffer, 1);
      data += width * 3;
    }
  }
  jpeg_finish_decompress(cinfo);
  return image;
}

} // namespace vlr
