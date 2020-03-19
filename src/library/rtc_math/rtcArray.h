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
 * file .......: rtcArray.h
 * authors ....: Benjamin Pitzer
 * organization: Robert Bosch LLC
 * creation ...: 08/16/2006
 * modified ...: $Date: 2009-01-21 18:19:16 -0800 (Wed, 21 Jan 2009) $
 * changed by .: $Author: benjaminpitzer $
 * revision ...: $Revision: 14 $
 */
#ifndef RTC_ARRAY_H
#define RTC_ARRAY_H

//== INCLUDES ==================================================================
#include "rtcMath.h"
#include "rtcVec.h"

//== NAMESPACES ================================================================
namespace rtc {

// Forward declarations
template <class T, int M> class Vec; // M-d vector
template <class T, int K> class Array; // Arbitrary-dimensional array
class Exception;

/**
 * A K-Dimensional Array of type T
 */
template <class T, int K>
class Array {
public:
  // Constructors/Destructor
  Array();
  Array(const Vec<int,K> dim);
  Array(const Array<T,K>& a);
  ~Array();

  // Mutators
  void deleteArray() { reset(); }
  void reset();
  void setSize(const Vec<int,K> dim_);
  void set(const T v);
  void set(const T* v, int i0 = 0, int vlen = -1);
  void set(const Array<T,K>& a);
  void addTo(const T* v, int i0 = 0, int vlen = -1);

  // Accessors
  Vec<int,K> size() const;
  int size(int i) const;
  int length() const;
  T& at(int i);
  T& at(const Vec<int,K> ind);
  const T& at(int i) const;
  const T& at(const Vec<int,K> ind) const;
  T operator () (int i) const;
  T& operator () (int i);
  T operator () (const Vec<int,K> ind) const;
  T& operator () (const Vec<int,K> ind);
  T operator [] (int i) const;
  T& operator [] (int i);
  T operator [] (const Vec<int,K> ind) const;
  T& operator [] (const Vec<int,K> ind);

  // Assignment operator
  Array<T,K>& operator = (const Array<T,K>& a);

  // Equality and inequality tests
  bool equalTo(const Array<T,K>& a) const;

  // Serialization
  bool write(OutputHandler& oh) const;
  bool read(InputHandler& ih);

  // Helper functions
  int indexOf(Vec<int,K> ind) const;

  // Data
  T* x; ///< storage array
protected:
  Vec<int,K> dim; ///< array dimensions
  Vec<int,K> mul; ///< multipliers for computing linear index
  int len; ///< linear length of array
};

// ASCII stream IO
template <class T, int K> std::ostream& operator << (std::ostream& os, const Array<T,K>& a);
template <class T, int K> std::istream& operator >> (std::istream& is, Array<T,K>& a);

//==============================================================================
// Array<T,K>
//==============================================================================

// Constructors/Destructor

/** Ctor that does no initalization.
 */
template <class T, int K>
inline Array<T,K>::Array() {
  x = NULL;
  len = 0;
  dim.set(0);
  mul.set(0);
}

/** Ctor that starts with given dimensions
 * @param dim_ are the sizes in the several dimension
 */
template <class T, int K>
inline Array<T,K>::Array(const Vec<int,K> dim_) {
  x=NULL;
  len = 0;
  dim.set(0);
  mul.set(0);
  setSize(dim_);
}

/** Copy Ctor
 * @param a the other vector
 */
template <class T, int K>
inline Array<T,K>::Array(const Array<T,K>& a) {
  x=NULL;
  len = 0;
  dim.set(0);
  mul.set(0);
  set(a);
}

/** Dtor that does no initalization.
 */
template <class T, int K>
inline Array<T,K>::~Array() {
  reset();
}

// Mutators

/** Deletes contents of array and frees memory
 */
template <class T, int K>
inline void Array<T,K>::reset() {
  len = 0;
  dim.set(0);
  mul.set(0);
  if (x) {
    delete [] x;
    x = NULL;
  }
}

/** Set the size of the array
 * @param dim_ the sizes of the array in each dimension
 */
template <class T, int K>
inline void Array<T,K>::setSize(const Vec<int,K> dim_) {
  if (!dim_.equalTo(dim)) {
    if (x != NULL) reset();
//    for (int k=0;k<K;k++) if (dim_(k) <= 0) {
//      if (AR_PO>1) {
//        std::cout << dim_;
//        std::cout << "Warning, negative or zero dimensions not allowed." << std::endl << std::flush;
//      }
//    }
    dim = dim_;
    len = dim.prod();
    mul(K-1) = 1;
    for (int d=1,k=K-2;d<K;d++,k--) mul(k) = mul(k+1)*dim(d);
    if (len>0) {
      x = new T[len];
    }
  }
}

/** Set the values of all the elements in the array to v
 * @param v is the value to set the array to
 */
template <class T, int K>
inline void Array<T,K>::set(const T v) {
  for (int i=0;i<len;i++) x[i] = v;
}

/** Set the values of all the elements in the array to v
 * @param v is the value to set the array to
 * @param i0 is offset where to start copying
 * @param vlen is the number of elements to copy
 */
template <class T, int K>
inline void Array<T,K>::set(const T* v, int i0, int vlen) {
  int i1 = (vlen == -1)? len:rtc_clamp(i0+vlen,0,len);
  for (int i=i0,j=0;i<i1;i++,j++) x[i] = v[j];
}

/** Set this array to the passed array, value-wise
 * @param a is the array to duplicate
 */
template <class T, int K>
inline void Array<T,K>::set(const Array<T,K>& a) {
  Vec<int,K> dim_ = a.Array<T,K>::size();
  Array<T,K>::setSize(dim_);
  Array<T,K>::set(a.x);
}

/** Add the values in array v to this array
 * @param v is the value to set the array to
 * @param i0 is offset where to start copying
 * @param vlen is the number of elements to copy
 */
template <class T, int K>
inline void Array<T,K>::addTo(const T* v, int i0, int vlen) {
  int i1 = (vlen == -1)? len:rtc_clamp(i0+vlen,0,len);
  for (int i=i0,j=0;i<i1;i++,j++) x[i] += v[j];
}

/** Returns mutable reference to array element
 * @param i is the index list
 * @return a reference to the array element
 */
template <class T, int K>
inline T& Array<T,K>::at(int i) {
  return x[i];
}

/** Returns mutable reference to array element
 * @param ind is the index list
 * @return a reference to the array element
 */
template <class T, int K>
inline T& Array<T,K>::at(const Vec<int,K> ind) {
  return x[indexOf(ind)];
}

/** Returns mutable reference to array element
 * @param i is the index list
 * @return a reference to the array element
 */
template <class T, int K>
inline const T& Array<T,K>::at(int i) const {
  return x[i];
}

/** Returns mutable reference to array element
 * @param ind is the index list
 * @return a reference to the array element
 */
template <class T, int K>
inline const T& Array<T,K>::at(const Vec<int,K> ind) const {
  return x[indexOf(ind)];
}

/** Returns mutable reference to array element
 * @param ind is the index list
 * @return a reference to the array element
 */
template <class T, int K>
inline T& Array<T,K>::operator () (const Vec<int,K> ind) {
  return x[indexOf(ind)];
}

/** Returns mutable reference to array element
 * @param i is the linear index
 * @return a reference to the array element
 */
template <class T, int K>
inline T& Array<T,K>::operator () (int i) {
#if AR_CHECK_BOUNDS
  if (i<0 || i>=len) {
    std::stringstream ss;
    ss << "Error. Index " << i << " exceeds bounds [0, ";
    ss << len << "]." << std::endl << std::flush;
    throw Exception(ss.str());
  }
#endif
  return x[i];
}

/** Returns array element
 * @param ind is the index list
 * @return array element
 */
template <class T, int K>
inline T Array<T,K>::operator () (const Vec<int,K> ind) const {
  return x[indexOf(ind)];
}

/** Returns array element
 * @param i is the linear index
 * @return array element
 */
template <class T, int K>
inline T Array<T,K>::operator () (int i) const {
#if AR_CHECK_BOUNDS
  if (i<0 || i>=len) {
    std::stringstream ss;
    ss << "Error. Index " << i << " exceeds bounds [0, ";
    ss << len << "]." << std::endl << std::flush;
    throw Exception(ss.str());
  }
#endif
  return x[i];
}

/** Returns mutable reference to array element
 * @param ind is the index list
 * @return a reference to the array element
 */
template <class T, int K>
inline T& Array<T,K>::operator [] (const Vec<int,K> ind) {
  return x[indexOf(ind)];
}

/** Returns mutable reference to array element
 * @param i is the linear index
 * @return a reference to the array element
 */
template <class T, int K>
inline T& Array<T,K>::operator [] (int i) {
#if AR_CHECK_BOUNDS
  if (i<0 || i>=len) {
    std::stringstream ss;
    ss << "Error. Index " << i << " exceeds bounds [0, ";
    ss << len << "]." << std::endl << std::flush;
    throw Exception(ss.str());
  }
#endif
  return x[i];
}

/** Returns array element
 * @param ind is the index list
 * @return array element
 */
template <class T, int K>
inline T Array<T,K>::operator [] (const Vec<int,K> ind) const {
  return x[indexOf(ind)];
}

/** Returns array element
 * @param i is the linear index
 * @return array element
 */
template <class T, int K>
inline T Array<T,K>::operator [] (int i) const {
#if AR_CHECK_BOUNDS
  if (i<0 || i>=len) {
    std::stringstream ss;
    ss << "Error. Index " << i << " exceeds bounds [0, ";
    ss << len << "]." << std::endl << std::flush;
    throw Exception(ss.str());
  }
#endif
  return x[i];
}

/** Set this array equal to passed array
 * @param a the array to replicate
 */
template <class T, int K>
inline Array<T,K>& Array<T,K>::operator = (const Array<T,K>& a) {
  set(a);
  return *this;
}

/** Compare the entire array to the passed array.
 * @returns true of all corresponding elements are the same
 */
template <class T, int K>
inline bool Array<T,K>::equalTo(const Array<T,K>& other) const {
  for (int i=0;i<len;i++)
    if (x[i]!=other.x[i])
      return(false);
  return true;
}

// Accessors

/** Returns vector of array dimensions
    @return Vec<int,K> of array dimension lengths
 */
template <class T, int K>
inline Vec<int,K> Array<T,K>::size() const {
  return dim;
}

/** Returns vector of array dimensions
    @return Vec<int,K> of array dimension lengths
 */
template <class T, int K>
inline int Array<T,K>::size(int i) const {
  return dim(i);
}

/** Returns linear length of data
    @return integer legth of data
 */
template <class T, int K>
inline int Array<T,K>::length() const {
  return len;
}

// Serialization routines

/** Write state to a binary stream.
 */
template <class T, int K>
inline bool Array<T,K>::write(OutputHandler& oh) const {
  dim.write(oh);
  if (len>0) oh.write((char *)(x),len*sizeof(T));
  return oh.good();
}

/** Restore state from a binary stream.
 */
template <class T, int K>
inline bool Array<T,K>::read(InputHandler& ih) {
  Vec<int,K> dim_;
  dim_.read(ih);
  if (!dim_.equalTo(Array<T,K>::size())) Array<T,K>::setSize(dim_);
  if (len>0) ih.read((char *)(x),len*sizeof(T));
  return ih.good();
}

/** Write state to stream as formated ASCII
 */
template <class T, int K>
std::ostream& operator << (std::ostream& os, const Array<T,K>& a) {
  int minFieldWidth = os.precision()+5;
  Vec<int,K> dim = a.template size();
  os << dim;
  for (int i=0;i<a.length();i++) {
    if (i>0 && i%dim(0) == 0) os << std::endl;
    if (K>2) if (i>0 && i%(dim(0)*dim(1)) == 0) os << std::endl;
    if (K>3) if (i>0 && i%(dim(0)*dim(1)*dim(2)) == 0) os << std::endl;
    os << std::setw(minFieldWidth) << a(i) << std::endl;
  }
  os << std::endl;
  return os;
}

/** Read state from stream of formated ASCII
 */
template <class T, int K>
std::istream& operator >> (std::istream& is, Array<T,K>& a) {
  Vec<int,K> dim;
  is >> dim; // read in dimensions
  if (!dim.equalTo(a.template Array<T,K>::size())) a.setSize(dim);
  for (int i=0;i<a.length();i++) {
    is >> a(i);
  }
  return is;
}

// Helper functions (used internally only)

/** Returns linear index of given array indices
 * @param ind is the vector of indices to be converted
 * @return the linear index in the data array x
 */
template <class T, int K>
inline int Array<T,K>::indexOf(Vec<int,K> ind) const {
#if AR_CHECK_BOUNDS
  for (int k=0;k<K;k++) if (ind(k) < 0 || ind(k) >= dim(k)) {
    std::stringstream ss;
    ss << "Error. Index " << ind(k) << " exceeds bounds [0, ";
    ss << dim(k) << "]";
    throw Exception(ss.str());
  }
#endif
  return (ind*mul).sum();
}

/**
* handler functions with standard storable interface
*/
template <class T, int K>
bool rtc_write(OutputHandler& oh, const Array<T,K>& data)
{
  return data.write(oh);
};

/**
* handler functions with standard storable interface
*/
template <class T, int K>
bool rtc_read(InputHandler& ih, Array<T,K>& data)
{
  return data.read(ih);
};

//==============================================================================
} // namespace rtc
//==============================================================================
#endif // ARRAY_H defined
//==============================================================================

