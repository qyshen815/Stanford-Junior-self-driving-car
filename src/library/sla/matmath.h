//-*-c++-*-
#ifndef MATMATH_H
#define MATMATH_H

/** 
 * @file matmath.h
 * @brief Basic static matrix vector types, templated in type and size
 * @author James R. Diebel, Stanford University
 *  
 * Set MATMATH_CHECK_BOUNDS define to enable or disable runtime bounds
 * checking.
 *
 * - History:
 *  - 31 July 2005 - Started (JD)
 *  - 30 Aug 2005 - Commented and tested (KH)
 */

#ifdef _DEBUG
 #define MATMATH_CHECK_BOUNDS 1
#else
 #define MATMATH_CHECK_BOUNDS 0
#endif

#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <mathutil.h>
#include <vec.h>
#include <vec2.h>
#include <vec3.h>
#include <vec4.h>
#include <mat.h>
#include <smat.h>
#include <smat2.h>
#include <smat3.h>
#include <smat4.h>
#include <miscutil.h>

#endif
