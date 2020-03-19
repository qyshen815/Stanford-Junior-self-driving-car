
/*
//
//             INTEL CORPORATION PROPRIETARY INFORMATION
//  This software is supplied under the terms of a license agreement or
//  nondisclosure agreement with Intel Corporation and may not be copied
//  or disclosed except in accordance with the terms of that agreement.
//     Copyright (c) 2001-2005 Intel Corporation. All Rights Reserved.
//
*/

#ifndef __JPEGIPP_H__
#define __JPEGIPP_H__

#include "jinclude.ipp.h"
#include "jpeglib.ipp.h"

#ifndef __IPPJ_H__
#include "ippj.h"
#endif


/* uncomment it for IPP v1.1 beta */
//#define IPP_11_BETA


//#define IPP_VER 11
/* uncomment it to use new functions in ippJP v 2.0 */
#define IPP_VER 20

/* We have changed some names after ippJP v1.1 Beta was released */
#ifdef IPP_11_BETA

#define IppiEncodeHuffmanState                   IppiEncoderHuffmanState
#define IppiDecodeHuffmanState                   IppiDecoderHuffmanState
#define IppiEncodeHuffmanSpec                    IppiEncoderHuffmanSpec
#define IppiDecodeHuffmanSpec                    IppiDecoderHuffmanSpec

#define ippiQuantFwdTableInit_JPEG_8u16u         ippiQuantFwdSpecInit_JPEG_8u16u
#define ippiEncodeHuffmanRawTableInit_JPEG_8u    ippiEncoderHuffmanRawSpecInit_JPEG_8u

#define ippiDecodeHuffmanSpecGetBufSize_JPEG_8u  ippiDecoderHuffmanSpecGetBufSize_JPEG_8u
#define ippiDecodeHuffmanSpecInit_JPEG_8u        ippiDecoderHuffmanSpecInit_JPEG_8u
#define ippiDecodeHuffmanStateGetBufSize_JPEG_8u ippiDecoderHuffmanStateGetBufSize_JPEG_8u
#define ippiDecodeHuffmanStateInit_JPEG_8u       ippiDecoderHuffmanStateInit_JPEG_8u
#define ippiDecodeHuffman8x8_JPEG_1u16s_C1       ippiDecoderHuffman8x8_JPEG_1u16s_C1
#define ippiEncodeHuffmanSpecInit_JPEG_8u        ippiEncoderHuffmanSpecInit_JPEG_8u
#define ippiEncodeHuffmanSpecGetBufSize_JPEG_8u  ippiEncoderHuffmanSpecGetBufSize_JPEG_8u
#define ippiEncodeHuffmanStateGetBufSize_JPEG_8u ippiEncoderHuffmanStateGetBufSize_JPEG_8u
#define ippiEncodeHuffmanStateInit_JPEG_8u       ippiEncoderHuffmanStateInit_JPEG_8u
#define ippiEncodeHuffman8x8_JPEG_16s1u_C1       ippiEncoderHuffman8x8_JPEG_16s1u_C1

#endif


/* Wrappers for Intel JPEG primitives */

/* encoder color conversion */
METHODDEF(void)
rgb_ycc_convert_intellib(
  j_compress_ptr cinfo,
  JSAMPARRAY     input_buf,
  JSAMPIMAGE     output_buf,
  JDIMENSION     output_row,
  int            num_rows);

METHODDEF(void)
rgb_gray_convert_intellib(
  j_compress_ptr cinfo,
  JSAMPARRAY     input_buf,
  JSAMPIMAGE     output_buf,
  JDIMENSION     output_row,
  int            num_rows);

METHODDEF(void)
cmyk_ycck_convert_intellib(
  j_compress_ptr cinfo,
  JSAMPARRAY     input_buf,
  JSAMPIMAGE     output_buf,
  JDIMENSION     output_row,
  int            num_rows);


/* forward DCT */
METHODDEF(void)
forward_DCT_intellib(
  j_compress_ptr       cinfo,
  jpeg_component_info* compptr,
  JSAMPARRAY           sample_data,
  JBLOCKROW            coef_blocks,
  JDIMENSION           start_row,
  JDIMENSION           start_col,
  JDIMENSION           num_blocks);

/* inverse DCT */
GLOBAL(void)
jpeg_idct_islow_intellib(
  j_decompress_ptr     cinfo,
  jpeg_component_info* compptr,
  JCOEFPTR             coef_block,
  JSAMPARRAY           output_buf,
  JDIMENSION           output_col);


LOCAL(void)
std_huff_tables_intellib(j_compress_ptr cinfo);


METHODDEF(void)
ycc_rgb_convert_intellib(
  j_decompress_ptr cinfo,
  JSAMPIMAGE       input_buf,
  JDIMENSION       input_row,
  JSAMPARRAY       output_buf,
  int              num_rows);

METHODDEF(void)
ycck_cmyk_convert_intellib(
  j_decompress_ptr cinfo,
  JSAMPIMAGE       input_buf,
  JDIMENSION       input_row,
  JSAMPARRAY       output_buf,
  int              num_rows);


METHODDEF(void)
h2v1_downsample_intellib(
  j_compress_ptr       cinfo,
  jpeg_component_info* compptr,
  JSAMPARRAY           input_data,
  JSAMPARRAY           output_data);


METHODDEF(boolean)
empty_output_buffer_intellib (j_compress_ptr cinfo);


LOCAL(void)
htest_one_block_intellib(
  j_compress_ptr cinfo,
  JCOEFPTR       block,
  int            last_dc_val,
  long           dc_counts[],
  long           ac_counts[]);

GLOBAL(void)
jpeg_gen_optimal_table_intellib(
  j_compress_ptr cinfo,
  JHUFF_TBL*     htbl,
  long           freq[]);


METHODDEF(boolean)
fill_input_buffer_intellib (j_decompress_ptr cinfo);


#endif /* __JPEGIPP_H__ */
