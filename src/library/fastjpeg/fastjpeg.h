#ifndef FASTJPEG_H
#define FASTJPEG_H

#include <jinclude.ipp.h>
#include <jpeglib.ipp.h>
#include <roadrunner.h>
#include <image.h>

  void BGRA2BGR_8u_C4C3R( const unsigned char* bgra, int bgra_step, 
			  unsigned char* bgr, int bgr_step,
			  int width, int height, int _swap_rb );
  void BGR2BGRA_8u_C3C4R( const unsigned char* src, int srcstep, 
			  unsigned char* dst, int dststep, 
			  int width, int height, int blue_idx );
  
  struct jpeg_decompress_struct* dgc_fastjpeg_init_decompress();
  struct jpeg_compress_struct* dgc_fastjpeg_init_compress();
  void dgc_fastjpeg_release( struct jpeg_decompress_struct **cinfo );
  
  vlr::dgc_image_t* dgc_fastjpeg_decompress_memory( struct jpeg_decompress_struct *cinfo, 
					       const unsigned char *jpeg_data, int jpeg_size,
					       vlr::dgc_image_t *dest_img, int *histogram );
  
  vlr::dgc_image_t* dgc_fastjpeg_decompress_crop_memory( struct jpeg_decompress_struct *cinfo, 
						    const unsigned char *jpeg_data, int jpeg_size,
						    int col_start, int row_start, 
						    int dest_width, int dest_height,
						    vlr::dgc_image_t* dest_img );
  
  void dgc_fastjpeg_decompress_crop_memory_start(struct jpeg_decompress_struct *cinfo, 
						 const unsigned char *jpeg_data, int jpeg_size,
						 int col_start, int row_start, 
						 int dest_width, int dest_height,
						 unsigned char* buffer );
  
  vlr::dgc_image_t* dgc_fastjpeg_decompress_file( struct jpeg_decompress_struct *cinfo, 
					  FILE *input_file, vlr::dgc_image_t* dest_img, int n_output_channels );
  
  /**
   * Quality is from 5..95
   **/
  void dgc_fastjpeg_compress_file( struct jpeg_compress_struct *cinfo, 
				   vlr::dgc_image_t* src_img, char *output_filename, int quality );
  
  
#endif
