/*
 * jmorecfg.h
 *
 * Copyright (C) 1991-1997, Thomas G. Lane.
 * This file is part of the Independent JPEG Group's software.
 * For conditions of distribution and use, see the accompanying README file.
 *
 * This file contains additional configuration options that customize the
 * JPEG software for special applications or support machine-dependent
 * optimizations.  Most users will not need to touch this file.
 */

/* jconfig.vc --- jconfig.h for Microsoft Visual C++ on Windows 95 or NT. */
/* see jconfig.doc for explanations */

#define HAVE_PROTOTYPES
#define HAVE_UNSIGNED_CHAR
#define HAVE_UNSIGNED_SHORT
/* #define void char */
/* #define const */
#undef CHAR_IS_UNSIGNED
#define HAVE_STDDEF_H
#define HAVE_STDLIB_IPP_H
#undef NEED_BSD_STRINGS
#undef NEED_SYS_TYPES_H
#undef NEED_FAR_POINTERS  /* we presume a 32-bit flat memory model */
#undef NEED_SHORT_EXTERNAL_NAMES
#undef INCOMPLETE_TYPES_BROKEN

#ifndef HAVE_BOOLEAN
/* Define "boolean" as unsigned char, not int, per Windows custom */
#ifndef __RPCNDR_H__    /* don't conflict if rpcndr.h already read */
typedef unsigned char boolean;
#endif
#endif
#define HAVE_BOOLEAN    /* prevent jmorecfg.h from redefining it */

#ifndef linux
#define USE_WINDOWS_MESSAGEBOX
#endif


//#undef USE_IPP
#define USE_IPP

#ifdef USE_IPP
#define IPPI_COPY
#define IPPS_ZERO
#define IPPJ_CC
#define IPPJ_SS
#define IPPJ_DCT_QNT
#define IPPJ_HUFF
#endif


#ifdef JPEG_INTERNALS

#undef RIGHT_SHIFT_IS_UNSIGNED

#endif /* JPEG_INTERNALS */

#ifdef JPEG_CJPEG_DJPEG

#define BMP_SUPPORTED   /* BMP image file format */
#define GIF_SUPPORTED   /* GIF image file format */
#define PPM_SUPPORTED   /* PBMPLUS PPM/PGM image file format */
#undef RLE_SUPPORTED    /* Utah RLE image file format */
#define TARGA_SUPPORTED   /* Targa image file format */

#define TWO_FILE_COMMANDLINE  /* optional */
#ifndef linux
#define USE_SETMODE   /* Microsoft has setmode() */
#endif
#undef NEED_SIGNAL_CATCHER
#undef DONT_USE_B_MODE
#undef PROGRESS_REPORT    /* optional */

#endif /* JPEG_CJPEG_DJPEG */
