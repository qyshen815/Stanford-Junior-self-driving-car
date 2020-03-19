/*
 * Copyright (C) 2009
 * Robert Bosch LLC
 * Research and Technology Center North America
 * Palo Alto, California
 *
 * All rights reserved.
 *
 *------------------------------------------------------------------------------
 * project ....: Autonomous Technologies
 * file .......: rtcArray2.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_ARRAY2_H
#define RTC_ARRAY2_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcVec2.h"
#include "rtcArray.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T, int K> class Array;  // K-dimensional array
template <class T> class Array2;        // 2-dimensional array
template <class T> class Vec2;          // 2d Vector

/**
 * A 2-Dimensional Array
 */
template <class T>
class Array2: public Array<T,2> {
public:
  // Constructors/Destructor
  Array2();
  Array2(int rows, int columns);
  Array2(int rows, int columns, const T* d);
  Array2(int rows, int columns, const T a);
  Array2(const Array<T,2>& a);

  // Mutators
  void setSize(int rows, int cols);

  // Accessors
  int rows() const;
  int columns() const;
  T& at(int rows, int columns);
  const T& at(int rows, int columns) const;
  T& operator () (int rows, int columns);
  T operator () (int rows, int columns) const;

  // Helper functions
  int indexOf(int rows, int columns) const;

  // inherit member data and functions of parent
  using Array<T,2>::x;
  using Array<T,2>::reset;
  using Array<T,2>::operator ();
  using Array<T,2>::size;
  using Array<T,2>::setSize;
  using Array<T,2>::at;
protected:
  // inherit member data and functions of parent
  using Array<T,2>::dim;
  using Array<T,2>::mul;
  using Array<T,2>::len;
};

// Declare a few common typdefs
typedef Array2<bool> Array2b;
typedef Array2<char> Array2c;
typedef Array2<unsigned char> Array2uc;
typedef Array2<int> Array2i;
typedef Array2<float> Array2f;
typedef Array2<double> Array2d;

//==============================================================================
// Array2<T>
//==============================================================================

// Constructors/Destructor

/** Ctor that does no initalization.
 */
template <class T>
inline Array2<T>::Array2() : Array<T,2>() {}

/** Ctor that starts with given dimensions
 * @param rows is the number of rows of a two-dimensional array
 * @param columns is the number of columns of a two-dimensional array
 */
template <class T>
inline Array2<T>::Array2(int rows, int columns) : Array<T,2>() {
  setSize(rows,columns);
}

/** Ctor that initializes elements from an array.
 * @param rows is the number of rows of a two-dimensional array
 * @param columns is the number of columns of a two-dimensional array
 * @param d pointer to the initalization array
 */
template <class T>
inline Array2<T>::Array2(int rows, int columns, const T* d) {
  setSize(rows,columns);
  set(d);
}

/** Ctor that initializes all elements from a scalar.
 * @param rows is the number of rows of a two-dimensional array
 * @param columns is the number of columns of a two-dimensional array
 * @param a the value to assign to all elements
 */
template <class T>
inline Array2<T>::Array2(int rows, int columns, const T a) {
  setSize(rows,columns);
  set(a);
}

/** Ctor that initializes an Array2<T> with a Array<T,2>.
 * @param a is the array to duplicate
 */
template <class T>
inline Array2<T>::Array2(const Array<T,2>& a) : Array<T,2>(a) {
}

// Mutators

/** Set the size of the array
 * @param rows is the number of rows of a two-dimensional array
 * @param columns is the number of columns of a two-dimensional array
 */
template <class T>
inline void Array2<T>::setSize(int rows, int columns) {
  Array<T,2>::setSize(Vec2i(rows,columns));
}

/** Returns mutable reference to array element
 * @param row is the row index
 * @param column is the column index
 * @return a reference to the array element
 */
template <class T>
inline T& Array2<T>::at(int row, int column) {
  return x[indexOf(row,column)];
}

/** Returns mutable reference to array element
 * @param row is the row index
 * @param column is the column index
 * @return a reference to the array element
 */
template <class T>
inline const T& Array2<T>::at(int row, int column) const {
  return x[indexOf(row,column)];
}

/** Returns mutable reference to array element
 * @param row is the row index
 * @param column is the column index
 * @return a reference to the array element
 */
template <class T>
inline T& Array2<T>::operator () (int row, int column) {
  return x[indexOf(row,column)];
}

/** Returns array element
 * @param row is the row index
 * @param column is the column index
 * @return array element
 */
template <class T>
inline T Array2<T>::operator () (int row, int column) const {
  return x[indexOf(row,column)];
}

/** Returns the number of rows
 * @return the number of rows
 */
template <class T>
inline int Array2<T>::rows() const {
  return Array<T,2>::dim[0];
}

/** Returns the number of columns
 * @return the number of columns
 */
template <class T>
inline int Array2<T>::columns() const {
  return Array<T,2>::dim[1];
}

// Helper functions (used internally only)

/** Returns linear index of given array indices
 * @param row is the row index
 * @param column is the column index
 * @return the linear index in the data array x
 */
template <class T>
inline int Array2<T>::indexOf(int row, int column) const {
#if AR_CHECK_BOUNDS
  if (row<0 || row>=dim(0) || column<0 || column>=dim(1)) {
    std::stringstream ss;
    ss << "Array2: Error. Indices (" << row << ", " << column;
    ss << ") exceed bounds [0, ";
    ss << dim(0) << "] or [0, " << dim(1) << "].";
    ss << std::endl << std::flush;
    throw Exception(ss.str());
  }
#endif
  return row*mul(0)+column;
}

//==============================================================================
} // namespace rtc
//==============================================================================
#endif // RTC_ARRAY2_H defined
//==============================================================================

