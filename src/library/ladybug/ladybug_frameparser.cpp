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
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================
#include <cassert>
#include <cstdlib>
#include <cstring>

//=============================================================================
// PGR Includes
//=============================================================================

//=============================================================================
// Project Includes
//=============================================================================
#include <stdio.h>
#include <ladybug_frameparser.h>

#define INVALID_ARGUMENT_IF_NULL( A ) \
   if( A == NULL ) \
   { \
      return LADYBUG_INVALID_ARGUMENT; \
   } \
   \
//=============================================================================
//Define the size of JPEG Header 
//=============================================================================
#define IMAGE_HEADER_SIZE               0x148
//=============================================================================
//JPEG Header 
//=============================================================================
unsigned char arImageHead[IMAGE_HEADER_SIZE] = {
     0xFF,0xD8,0xFF,0xE0,0x00,0x10,0x4A,0x46, /* 000 */
     0x49,0x46,0x00,0x01,0x01,0x00,0x00,0x01, /* 008 */
     0x00,0x01,0x00,0x00,0xff,0xdb,0x00,0x43, /* 010 */
     0x00,0x0d,0x09,0x0a,0x0b,0x0a,0x08,0x0d, /* 018 : 19: Quantize */
     0x0b,0x0a,0x0b,0x0e,0x0e,0x0d,0x0f,0x13, /* 020 */
     0x20,0x15,0x13,0x12,0x12,0x13,0x27,0x1c, /* 028 */
     0x1e,0x17,0x20,0x2e,0x29,0x31,0x30,0x2e, /* 030 */
     0x29,0x2d,0x2c,0x33,0x3a,0x4a,0x3e,0x33, /* 038 */
     0x36,0x46,0x37,0x2c,0x2d,0x40,0x57,0x41, /* 040 */
     0x46,0x4c,0x4e,0x52,0x53,0x52,0x32,0x3e, /* 048 */
     0x5a,0x61,0x5a,0x50,0x60,0x4a,0x51,0x52, /* 050 */
     0x4f,0xff,0xc0,0x00,0x0b,0x08,0x00,0x20, /* 058 : 5E: HT (be) */
     0x00,0x20,0x01,0x01,0x11,0x00,0xFF,0xC4, /* 060 : 60: WD (be) */
     0x00,0x1F,0x00,0x00,0x01,0x05,0x01,0x01, /* 068 */
     0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00, /* 070 */
     0x00,0x00,0x00,0x00,0x01,0x02,0x03,0x04, /* 078 */
     0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0xFF, /* 080 */
     0xC4,0x00,0xB5,0x10,0x00,0x02,0x01,0x03, /* 088 */
     0x03,0x02,0x04,0x03,0x05,0x05,0x04,0x04, /* 090 */
     0x00,0x00,0x01,0x7D,0x01,0x02,0x03,0x00, /* 098 */
     0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06, /* 0A0 */
     0x13,0x51,0x61,0x07,0x22,0x71,0x14,0x32, /* 0A8 */
     0x81,0x91,0xA1,0x08,0x23,0x42,0xB1,0xC1, /* 0B0 */
     0x15,0x52,0xD1,0xF0,0x24,0x33,0x62,0x72, /* 0B8 */
     0x82,0x09,0x0A,0x16,0x17,0x18,0x19,0x1A, /* 0C0 */
     0x25,0x26,0x27,0x28,0x29,0x2A,0x34,0x35, /* 0C8 */
     0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45, /* 0D0 */
     0x46,0x47,0x48,0x49,0x4A,0x53,0x54,0x55, /* 0D8 */
     0x56,0x57,0x58,0x59,0x5A,0x63,0x64,0x65, /* 0E0 */
     0x66,0x67,0x68,0x69,0x6A,0x73,0x74,0x75, /* 0E8 */
     0x76,0x77,0x78,0x79,0x7A,0x83,0x84,0x85, /* 0F0 */
     0x86,0x87,0x88,0x89,0x8A,0x92,0x93,0x94, /* 0F8 */
     0x95,0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3, /* 100 */
     0xA4,0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xB2, /* 108 */
     0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA, /* 110 */
     0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9, /* 118 */
     0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8, /* 120 */
     0xD9,0xDA,0xE1,0xE2,0xE3,0xE4,0xE5,0xE6, /* 128 */
     0xE7,0xE8,0xE9,0xEA,0xF1,0xF2,0xF3,0xF4, /* 130 */
     0xF5,0xF6,0xF7,0xF8,0xF9,0xFA,0xFF,0xDA, /* 138 */
     0x00,0x08,0x01,0x01,0x00,0x00,0x3F,0x00, /* 140 */
};


static __inline unsigned
swab( unsigned num )
{
  return (( num>>24) |
	  ( ( num & 0x00FF0000 ) >> 8 ) |
	  ( ( num & 0x0000FF00 ) << 8 ) |
	  (  num << 24 ) );
}


LadybugCompressorHeaderInfo::LadybugCompressorHeaderInfo()
{
  m_iTotalSize = 0;
  m_iNumImages = 0;
  memset( &m_arInfo, 0x0, sizeof( ImageInfo ) * LADYBUG_COMPRESSOR_HEADER_MAX_IMAGES );
  
  m_pDataStart = NULL;
}

LadybugCompressorHeaderInfo::~LadybugCompressorHeaderInfo()
{
}



int   
LadybugCompressorHeaderInfo::totalSize() const
{
  return m_iTotalSize;
}

int   
LadybugCompressorHeaderInfo::images() const
{
  return m_iNumImages;
}

LadybugError 
LadybugCompressorHeaderInfo::parse( const unsigned char*   pData, 
				    unsigned int           cols, 
				    unsigned int           rows )
{
  INVALID_ARGUMENT_IF_NULL( pData );
  
  m_iTotalSize = 0;
  m_iNumImages = 0;
  memset( &m_arInfo, 0x0, sizeof( ImageInfo ) * LADYBUG_COMPRESSOR_HEADER_MAX_IMAGES );
  
  m_pDataStart = pData;
  
  //
  // Revisit: the header at offset 16 is the same as LadybugImageInfo
  // version 2.
  //
  
  unsigned signature = 
    swab( *(unsigned*)&pData[ 16 + 0 ] );
  
  unsigned version  = swab( *(unsigned*)&pData[ 16 + 4 ] );
  //unsigned sequence = swab( *(unsigned*)&pData[ 16 + 16 ] );
  
  if( signature != 0xCAFEBABE && version != 2 ) {
    fprintf( stderr, "\nno cafebabe found\n" );
    return LADYBUG_JPEG_ERROR;
  } else {
  }
  
  
  for( int i = 0; i < LADYBUG_COMPRESSOR_HEADER_MAX_IMAGES; i++ ) {
    unsigned offset = 
      swab( *(unsigned*)( pData + 1024 - ( 24 - i ) * 8 + 0 ) );
    unsigned size = 
      swab( *(unsigned*)( pData + 1024 - ( 24 - i ) * 8 + 4 ) );
    
    m_arInfo[ i ].offset = offset;
    m_arInfo[ i ].size   = size;
    
    if( size == 0 ) {
      if ( offset == 0 ) {
	return LADYBUG_JPEG_INCOMPLETE_COMPRESSION;
      } else {
	return LADYBUG_JPEG_BUFFER_TOO_SMALL;
      }
    } else {
      m_iNumImages++;
      
      if( offset + size > (unsigned)m_iTotalSize ) {
	m_iTotalSize = offset + size;
      }
      
      m_arInfo[ i ].pData = pData + offset;
      
      m_arInfo[ i ].jpegCols = 
	( m_arInfo[ i ].pData[ 0x60 ] << 8 ) + m_arInfo[ i ].pData[ 0x61 ];
      
      m_arInfo[ i ].jpegRows = 
	( m_arInfo[ i ].pData[ 0x5E ] << 8 ) + m_arInfo[ i ].pData[ 0x5F ];
      
      //Check 
      if( cols != 0 ) {
	if( m_arInfo[ i ].jpegCols != cols ) {
	  fprintf(stderr, "\nERROR: jpegCols=%d, cols=%d     \n", 
		  m_arInfo[ i ].jpegCols, cols );
	  // jpeg is wrong size.
	  assert( false );
	  return LADYBUG_JPEG_HEADER_ERROR;
	}
      }
      
      if( rows != 0 ) {
	if( m_arInfo[ i ].jpegRows != rows ) {
	  fprintf(stderr, "\nERROR: jpegRows=%d, rows=%d\n    ", 
		  m_arInfo[ i ].jpegRows, rows );
	  // jpeg is wrong size.
	  assert( false );
	  return LADYBUG_JPEG_HEADER_ERROR;
	}
      }
    }
  }
  //   fprintf( stderr, "total size=%8d\n", m_iTotalSize);
  
  if( m_iNumImages == 0 ) {
    return LADYBUG_JPEG_NO_IMAGE;
  }
  
  return LADYBUG_OK;
}

 
const LadybugCompressorHeaderInfo::ImageInfo* 
LadybugCompressorHeaderInfo::getInfo( int index ) const
{
  if( index >= LADYBUG_COMPRESSOR_HEADER_MAX_IMAGES ) {
    assert( false );
    return NULL;
  }
  
  return &m_arInfo[ index ];
}

