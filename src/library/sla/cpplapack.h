//-*-c++-*-
#ifndef CPPLAPACK_H
#define CPPLAPACK_H

/** 
 * @file cpplapack.hh
 * @brief Wrapper header to include clapack.h with extern "C"
 * @author James R. Diebel, Stanford University
 *  
 * - History:
 *  - 21 March 2006 - Started 
 */


extern "C" {
#include <myf2c.h>
#include <cblas.h>
#include <clapack.h>
}

#endif
