//=============================================================================
// Copyright ? 2004 Point Grey Research, Inc. All Rights Reserved.
// 
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with Point Grey Research, Inc. (PGR).
// 
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

#ifndef __LADYBUG_PTGREY_H__
#define __LADYBUG_PTGREY_H__

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * An enumeration  of all the possible errors returned by the Ladybug API.
 */
typedef enum LadybugError
{
  /** Function completed successfully. */
  LADYBUG_OK,
  /** Invalid argument passed. */
  LADYBUG_INVALID_ARGUMENT,
  /** An error occured during JPEG decompression. */
  LADYBUG_JPEG_ERROR,
  /** An error occured in JPEG image buffer header  */
  LADYBUG_JPEG_HEADER_ERROR,
  /** JPEG image buffer is too small to hold image data  */
  LADYBUG_JPEG_BUFFER_TOO_SMALL,
  /** The compressor did not have enough time to finish compressing the data  */
  LADYBUG_JPEG_INCOMPLETE_COMPRESSION,
  /** There is no image in this frame  */
  LADYBUG_JPEG_NO_IMAGE,
  
} LadybugError;

#define LADYBUG_COMPRESSOR_HEADER_MAX_IMAGES   24

/**
 * Handles parsing of the Compressor jpeg header.
 * 
 */
class LadybugCompressorHeaderInfo
{
public:

  struct ImageInfo {

    unsigned             size;
    unsigned             offset;
    bool                 bSkipped;
    unsigned             jpegRows;
    unsigned             jpegCols;
    
    const unsigned char* pData;

  };
  
  
  LadybugCompressorHeaderInfo();
  virtual ~LadybugCompressorHeaderInfo();
  
  /** 
   * Parse the given data buffer. 
   *
   * @param cols Columns in expected jpeg images. Ignored if zero.
   * @param Rows Rows in expected jpeg images. Ignored if zero.
   *
   */
  LadybugError parse( const unsigned char* pData, unsigned int cols, unsigned int rows );
  
  
  /** @return Image info structure corresponding to given index. */
  const ImageInfo* getInfo( int index ) const;
  
  /** @return the total size, in bytes, of the jpeg data. */
  int totalSize() const;
  
  /** @return Number of images in this compressor jpeg buffer. */
  int images() const;
  
  const unsigned char* m_pDataStart;
  
  
private:

   int         m_iTotalSize;
   int         m_iNumImages;
   ImageInfo   m_arInfo[LADYBUG_COMPRESSOR_HEADER_MAX_IMAGES];

};


#ifdef __cplusplus
}
#endif
  
#endif // #ifndef __COMPRESSORHEADERINFO_H__
