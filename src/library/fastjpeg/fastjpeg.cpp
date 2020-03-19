/*
* JPEG Routines sped up by IPP
*/

#include "fastjpeg.h"
#include <jerror.ipp.h>   /* get library error codes too */

#include <ctype.h>    /* to declare isprint() */
#ifdef HAVE_IPP
#include <ippcore.h>
#endif
#include <setjmp.h>

#ifdef DEBUG_DISPLAY
#endif

using namespace vlr;

struct my_error_mgr {
  struct jpeg_error_mgr pub;    /* "public" fields */

  jmp_buf setjmp_buffer;        /* for return to caller */
};

typedef struct my_error_mgr * my_error_ptr;

static struct my_error_mgr jerr;

// ****************************************
// ********* Memory Source Stream *********
// ****************************************
typedef struct {
  struct jpeg_source_mgr pub; /* public fields */

  JOCTET * buffer;    /* start of buffer */
  int buffer_size;
} mem_source_mgr;

typedef mem_source_mgr * mem_src_ptr;

/*
 * Initialize source --- called by jpeg_read_header
 * before any data is actually read.
 */

static void 
init_source (j_decompress_ptr /*cinfo*/) { }


/*
 * Fill the input buffer --- called whenever buffer is emptied.
 */

static boolean 
fill_input_buffer (j_decompress_ptr /*cinfo*/) {
//  fprintf(stderr, "JPEG Buffer empty, corrupt jpeg?\n" );
//  ERREXIT(cinfo, JERR_INPUT_EMPTY);
  return 0;
}

/*
 * Skip data --- used to skip over a potentially large amount of
 * uninteresting data (such as an APPn marker).
 */

static void skip_input_data (j_decompress_ptr cinfo, long num_bytes) {
  mem_src_ptr src = (mem_src_ptr) cinfo->src;

  src->pub.next_input_byte += (size_t) num_bytes;
  src->pub.bytes_in_buffer -= (size_t) num_bytes;
}


/*
 * An additional method that can be provided by data source modules is the
 * resync_to_restart method for error recovery in the presence of RST markers.
 * For the moment, this source module just uses the default resync method
 * provided by the JPEG library.  That method assumes that no backtracking
 * is possible.
 */


/*
 * Terminate source --- called by jpeg_finish_decompress
 * after all data has been read.  Often a no-op.
 *
 * NB: *not* called by jpeg_abort or jpeg_destroy; surrounding
 * application must deal with any cleanup that should happen even
 * for error exit.
 */

static void 
term_source (j_decompress_ptr /*cinfo*/) {}

void 
jpeg_mem_src (j_decompress_ptr cinfo, const unsigned char* mem, int mem_size) 
{
  mem_src_ptr src;

  /* The source object and input buffer are made permanent so that a series
   * of JPEG images can be read from the same file by calling jpeg_stdio_src
   * only before the first one.  (If we discarded the buffer at the end of
   * one image, we'd likely lose the start of the next one.)
   * This makes it unsafe to use this manager and a different source
   * manager serially with the same JPEG object.  Caveat programmer.
   */
  if (cinfo->src == NULL) { /* first time for this JPEG object? */
    cinfo->src = (struct jpeg_source_mgr *)
      (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,
          SIZEOF(mem_source_mgr));
    src = (mem_src_ptr) cinfo->src;
  }

  src = (mem_src_ptr) cinfo->src;
  src->buffer = (JOCTET*)mem;
  src->buffer_size = mem_size;
  src->pub.bytes_in_buffer = mem_size;
  src->pub.next_input_byte = mem;

  src->pub.init_source = init_source;
  src->pub.fill_input_buffer = fill_input_buffer;
  src->pub.skip_input_data = skip_input_data;
  src->pub.resync_to_restart = jpeg_resync_to_restart; /* use default method */
  src->pub.term_source = term_source;
}

void 
BGR2RGB_8u_C3R( const unsigned char* bgr, int bgr_step, 
		unsigned char* rgb, int rgb_step, 
		int width, int height )
{
  int i;
  for( ; height--; ) {
    for( i = 0; i < width; i++, bgr += 3, rgb += 3 ) {
      unsigned char t0 = bgr[0], t1 = bgr[1], t2 = bgr[2];
      rgb[2] = t0; rgb[1] = t1; rgb[0] = t2;
    }
    bgr += bgr_step - width*3;
    rgb += rgb_step - width*3;
  }
}

void 
BGRA2BGR_8u_C4C3R( const unsigned char* bgra, int bgra_step, 
		   unsigned char* bgr, int bgr_step, 
		   int width, int height, int _swap_rb )
{
  int i;
    int swap_rb = _swap_rb ? 2 : 0;
    for( ; height--; )
    {
        for( i = 0; i < width; i++, bgr += 3, bgra += 4 )
        {
            unsigned char t0 = bgra[swap_rb], t1 = bgra[1];
            bgr[0] = t0; bgr[1] = t1;
            t0 = bgra[swap_rb^2]; bgr[2] = t0;
        }
        bgr += bgr_step - width*3;
        bgra += bgra_step - width*4;
    }
}

void 
BGR2BGRA_8u_C3C4R( const unsigned char* src, int srcstep, 
		   unsigned char* dst, int dststep,
		   int width, int height, int blue_idx ) 
{
  int i;
  
  srcstep /= sizeof(src[0]);
  dststep /= sizeof(dst[0]);
  srcstep -= width*3;
  width *= 4;
  
  for( ; height--; src += srcstep, dst += dststep ) {
    for( i = 0; i < width; i += 4, src += 3 ) {
      unsigned char t0=src[blue_idx], t1=src[1], t2=src[blue_idx^2];    
      dst[i] = t0;
      dst[i+1] = t1;
      dst[i+2] = t2;
      dst[i+3] = 0;
    } 
  }
}

// ****************************************
// ********** Fast JPEG Library ***********
// ****************************************

/*
 * Here's the routine that will replace the standard error_exit method:
 */

void my_jpeg_error_exit (j_common_ptr cinfo)
{
  /* cinfo->err really points to a my_error_mgr struct, so coerce pointer */
  my_error_ptr myerr = (my_error_ptr) cinfo->err;

  /* Always display the message. */
  /* We could postpone this until after returning, if we chose. */
  fprintf( stderr, "jpeg error:");
  (*cinfo->err->output_message) (cinfo);

  /* Return control to the setjmp point */
  longjmp(myerr->setjmp_buffer, 1);
}


struct jpeg_decompress_struct* 
dgc_fastjpeg_init_decompress( void ) 
{
  struct jpeg_decompress_struct* cinfo = (jpeg_decompress_struct*)calloc( sizeof(struct jpeg_decompress_struct), 1);
  
  if(ippStsNoErr > ippStaticInit()) {
    fprintf(stderr,"Can't initialize IPP library\n");
    exit(EXIT_FAILURE);
  }
  
  /* Initialize the JPEG decompression object with default error handling. */
  cinfo->err = jpeg_std_error((struct jpeg_error_mgr*)&jerr);
  cinfo->err->error_exit = my_jpeg_error_exit;
  
  jpeg_create_decompress(cinfo);
  cinfo->do_block_smoothing = 0;
  
  return cinfo;
}

struct jpeg_compress_struct* 
dgc_fastjpeg_init_compress( void ) 
{
  struct jpeg_compress_struct* cinfo = (jpeg_compress_struct*)calloc( sizeof(struct jpeg_compress_struct), 1);

  if(ippStsNoErr > ippStaticInit()) {
    fprintf(stderr,"Can't initialize IPP library\n");
    exit(EXIT_FAILURE);
  }

  /* Initialize the JPEG decompression object with default error handling. */
  cinfo->err = jpeg_std_error((struct jpeg_error_mgr*)&jerr);
  cinfo->err->error_exit = my_jpeg_error_exit;

  jpeg_create_compress(cinfo); 

  return cinfo;
}


void 
dgc_fastjpeg_release( struct jpeg_decompress_struct **cinfo ) {
  jpeg_destroy_decompress(*cinfo);
  *cinfo = NULL;
}


dgc_image_t*
dgc_fastjpeg_decompress_memory( struct jpeg_decompress_struct *cinfo, 
				const unsigned char *jpeg_data, int jpeg_size,
				dgc_image_t* dest_img, int *histogram )
{
  int src_width, src_height, i;
  unsigned char *dest_data, *ws;
  
  cinfo->crop_start_col = -1;
  cinfo->crop_end_col = -1;
  cinfo->crop_start_row = -1;
  cinfo->crop_end_row = -1;

  /* Establish the setjmp return context for my_error_exit to use. */
  if (setjmp(jerr.setjmp_buffer)) {
    /* If we get here, the JPEG code has signaled an error.
     * We need to clean up the JPEG object, close the input file, and return.
     */
    jpeg_abort_decompress( cinfo);
    fprintf( stderr, "Aborted jpeg compression\n" );
    return 0;
  }

 /* Specify data source for decompression */
  jpeg_mem_src(cinfo, jpeg_data, jpeg_size);
  
  /* Read file header, set default decompression parameters */
  jpeg_read_header(cinfo, TRUE);
  
  /* Start decompressor */
  jpeg_start_decompress(cinfo);
  src_width  = cinfo->image_width;
  src_height = cinfo->image_height;

  if( dest_img == NULL) {
    dest_img = (dgc_image_t*) malloc( sizeof(dgc_image_t) );
    dgc_test_alloc(dest_img);
    dest_img->nchannels = cinfo->output_components;
    dest_img->width = src_width;
    dest_img->height = src_height;
    dest_img->pix = (unsigned char *) malloc( dest_img->width * 
					      dest_img->height * 
					      dest_img->nchannels * 
					      sizeof(unsigned char) );
    dgc_test_alloc(dest_img->pix);
  } else {
    if( dest_img->width != src_width || dest_img->height != src_height ) {
      dgc_die("fastjpeg_decompress: dest image has size %dx%d not %dx%d\n",
	      dest_img->width, dest_img->height, src_width, src_height );
    }
  }
  dest_data = (unsigned char*)dest_img->pix;
    
  if( histogram == NULL ) {
    int32_t width = dest_img->width;
    for( i = 0; i < src_height; i++ ) {
      jpeg_read_scanlines( cinfo, &dest_data, 1 );
      dest_data += width;
    } 
  } else {
    for( i = 0; i < src_height; i++ ) {
      jpeg_read_scanlines( cinfo, &dest_data, 1 );
      if( i%8 == 0 ) {
        ws = dest_data + dest_img->width;
        for( ; dest_data < ws; dest_data+=8 )
          histogram[ dest_data[0] ]++;
      } else {
        dest_data += dest_img->width;
      }
    } 
  }
  jpeg_finish_decompress( cinfo );
  
  return dest_img;
}

dgc_image_t*
dgc_fastjpeg_decompress_crop_memory( struct jpeg_decompress_struct *cinfo, 
				     const unsigned char *jpeg_data, int jpeg_size,
				     int col_start, int row_start, 
				     int dest_width, int dest_height,
				     dgc_image_t* dest_img ) 
{
  int src_width, src_height, i;
  unsigned char *dest_data;
  unsigned char *buffer;
  JSAMPARRAY bufferptr = 0;
  cinfo->crop_start_col = 8 * (col_start / 8);
  cinfo->crop_end_col = col_start + dest_width - 1;
  cinfo->crop_start_row = 8 * (row_start / 8);
  cinfo->crop_end_row = row_start + dest_height - 1;
  
  /* Establish the setjmp return context for my_error_exit to use. */
  if (setjmp(jerr.setjmp_buffer)) {
    /* If we get here, the JPEG code has signaled an error.
     * We need to clean up the JPEG object, close the input file, and return.
     */
    jpeg_abort_decompress( cinfo);
    fprintf( stderr, "Aborted jpeg compression\n" );
    return 0;
  }

 /* Specify data source for decompression */
  jpeg_mem_src(cinfo, jpeg_data, jpeg_size);
  
  /* Read file header, set default decompression parameters */
  jpeg_read_header(cinfo, TRUE);
  
  /* Start decompressor */
  jpeg_start_decompress(cinfo);
  src_width = cinfo->image_width;
  src_height = cinfo->image_height;
  
  if( dest_img == NULL) {
    dest_img = (dgc_image_t*) malloc( sizeof(dgc_image_t) );
    dgc_test_alloc(dest_img);
    dest_img->nchannels = 1;
    dest_img->width = dest_width;
    dest_img->height = dest_height;
    dest_img->pix = (unsigned char *) malloc( dest_img->width * 
					      dest_img->height * 
					      dest_img->nchannels * 
					      sizeof(unsigned char) );
    dgc_test_alloc(dest_img->pix);
  }
  dest_data = (unsigned char*)dest_img->pix;

  buffer = (unsigned char*)malloc( src_width );
  bufferptr = &buffer;

  for( i = 0; i < src_height; i++ ) {
    if( i >= row_start + dest_height )
      break;
    jpeg_read_scanlines( cinfo, bufferptr, 1 );
    if( i >= row_start ) {
      memcpy( dest_data, &buffer[col_start], dest_width);
      dest_data += (dest_width * dest_img->nchannels);
    }
  } 
  if( i < src_height ) {
    jpeg_abort_decompress( cinfo );
  } else
    jpeg_finish_decompress( cinfo );
  
  free( buffer );

  return dest_img;
}

void 
dgc_fastjpeg_decompress_crop_memory_start(struct jpeg_decompress_struct *cinfo, 
					  const unsigned char *jpeg_data, int jpeg_size,
					  int col_start, int row_start, 
					  int dest_width, int dest_height,
					  unsigned char *buffer ) 
{
  int src_width, src_height, i;
  JSAMPARRAY bufferptr = 0;
  cinfo->crop_start_col = 8 * (col_start / 8);
  cinfo->crop_end_col = col_start + dest_width - 1;
  cinfo->crop_start_row = 8 * (row_start / 8);
  cinfo->crop_end_row = row_start + dest_height - 1;
  
  /* Establish the setjmp return context for my_error_exit to use. */
  if (setjmp(jerr.setjmp_buffer)) {
    /* If we get here, the JPEG code has signaled an error.
     * We need to clean up the JPEG object, close the input file, and return.
     */
    jpeg_abort_decompress( cinfo);
    fprintf( stderr, "Aborted jpeg compression\n" );
    return;
  }

 /* Specify data source for decompression */
  jpeg_mem_src(cinfo, jpeg_data, jpeg_size);
  
  /* Read file header, set default decompression parameters */
  jpeg_read_header(cinfo, TRUE);
  
  /* Start decompressor */
  jpeg_start_decompress(cinfo);
  src_width = cinfo->image_width;
  src_height = cinfo->image_height;

//  buffer = (unsigned char*)malloc( src_width );
  bufferptr = &buffer;

  for( i = 0; i < row_start; i++ ) {
    jpeg_read_scanlines( cinfo, bufferptr, 1 );
  } 
}

dgc_image_t*
dgc_fastjpeg_decompress_file( struct jpeg_decompress_struct *cinfo, 
			      FILE *input_file, dgc_image_t* dest_img, 
			      int n_output_channels ) 
{
  int width, height;
  unsigned char *data;
  JSAMPARRAY buffer = 0;
  cinfo->crop_start_col = -1;
  cinfo->crop_end_col = -1;
  cinfo->crop_start_row = -1;
  cinfo->crop_end_row = -1;

 /* Specify data source for decompression */
  jpeg_stdio_src(cinfo, input_file);
  
  /* Read file header, set default decompression parameters */
  jpeg_read_header(cinfo, TRUE);
  
  /* Start decompressor */
  jpeg_start_decompress(cinfo);
  width = cinfo->image_width;
  height = cinfo->image_height;
  
  if( dest_img == NULL) {
    dest_img = (dgc_image_t*) malloc( sizeof(dgc_image_t) );
    dgc_test_alloc(dest_img);
    dest_img->nchannels = n_output_channels;
    dest_img->width = width;
    dest_img->height = height;
    dest_img->pix = (unsigned char *) malloc( dest_img->width * 
					      dest_img->height * 
					      dest_img->nchannels * 
					      sizeof(unsigned char) );
    dgc_test_alloc(dest_img->pix);
  }
  data = (unsigned char*)dest_img->pix;

  if( n_output_channels == cinfo->num_components ) {
    for( ; height--; ) {
      buffer = &data;
      jpeg_read_scanlines( cinfo, buffer, 1 );
      int i,i3;
      for( i=0, i3=0; i < width; i++, i3+=3 ) {
        unsigned char tmp = data[i3];
        data[i3]  = data[i3+2];
        data[i3+2]= tmp;
      }
      data += (dest_img->width*dest_img->nchannels);
    }
  } else {
    uint8_t* temp_data = (uint8_t*)malloc( width * n_output_channels );
    buffer = &temp_data;
    if( cinfo->num_components == 3 && n_output_channels == 4 ) {
      for( ; height--; ) {
        jpeg_read_scanlines( cinfo, buffer, 1 );
        BGR2BGRA_8u_C3C4R((const uint8_t*)buffer, width*3, data,
			   (dest_img->width*dest_img->nchannels), 512, 1, 2);
        data += (dest_img->width*dest_img->nchannels);
      }
    } else {
      dgc_die("unsupported format: n_output_channels=%d and %d channels in jpeg image\n", 
              n_output_channels, cinfo->num_components); 
    }
    free(temp_data);
  }

  jpeg_finish_decompress(cinfo);
  
  return dest_img;
}

void 
dgc_fastjpeg_compress_file( struct jpeg_compress_struct *cinfo, 
			    dgc_image_t *src_img, char *output_filename, int quality ) 
{
  FILE *output_file = fopen(output_filename, "wb");
  if( output_file == NULL )
    dgc_die("fastjpeg_compress_file: cannot open file %s for writing\n", output_filename);
  jpeg_stdio_dest(cinfo, output_file);

  cinfo->image_width = src_img->width;
  cinfo->image_height = src_img->height;
  switch( src_img->nchannels ) {
  case 3: 
  case 4: 
    cinfo->in_color_space = JCS_RGB;
    cinfo->input_components = 3;
    break;
  case 1:
    cinfo->in_color_space = JCS_GRAYSCALE; 
    cinfo->input_components = 1;
    break;
  default:
    dgc_die("dgc_fastjpeg_compress_file does not support %d color channels yet\n", src_img->nchannels);
  }

  jpeg_set_defaults(cinfo);
  jpeg_set_quality( cinfo, quality, TRUE /* limit to baseline-JPEG values */ );
  /* Start compressor */
  jpeg_start_compress(cinfo, TRUE);

  /* Process data */
  uint8_t tmp[src_img->width*src_img->nchannels];
  while (cinfo->next_scanline < cinfo->image_height) {
    uint8_t* buffer = &src_img->pix[ cinfo->next_scanline * src_img->width * src_img->nchannels];
    
    uint8_t* ptr = buffer;
    if( src_img->nchannels == 3 ) {
      BGR2RGB_8u_C3R((const uint8_t*)buffer, 0, (uint8_t*)tmp, 0, src_img->width,1 );
      ptr = tmp;
    }
    else if( src_img->nchannels == 4 ) {
      BGRA2BGR_8u_C4C3R((const uint8_t*) buffer, 0, (uint8_t*)tmp, 0, src_img->width, 1, 2 );
      ptr = tmp;
    }

    jpeg_write_scanlines(cinfo, (JSAMPLE**)&ptr, 1 );
  }
 
  /* Finish compression and release memory */
  jpeg_finish_compress(cinfo);
  fclose(output_file);
}

