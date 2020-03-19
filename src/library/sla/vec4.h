//-*-c++-*-
#ifndef VEC4_H
#define VEC4_H

/** 
 * @file vec4.h
 * @brief Basic static 4d vector type, templated in type
 * @author James R. Diebel, Stanford University
 *  
 * - History:
 *  - 31 July 2005 - Started (JD)
 *  - 30 Aug 2005 - Commented and tested (KH)
 *  - 30 Aug 2005 forked off from matmath.h
 */

#include <mathutil.h>
#include <vec.h>

/**
 * @namespace sla
 * @brief Simple Linear Algebra - library of matrix and vector classes
 */
namespace sla {

  /////////////////////////////////////////////////////////////////////////////
  // DECLARATIONS
  /////////////////////////////////////////////////////////////////////////////

  // Forward declarations
  template <class T, int M> class Vec; // M-d vector
  template <class T> class Vec4; // 4d Vector

  //////////////////// Vec4 ////////////////////
  /**
   * A 4-D vector.
   */
  template <class T> 
  class Vec4: public Vec<T,4> {
  public:
    // inherit member data and functions of parent
    using Vec<T,4>::x;
    using Vec<T,4>::set;

    // Constructors
    Vec4();
    Vec4(const T* d);
    Vec4(const T a);
    Vec4(const T x0, const T x1, const T x2, const T x3);
    Vec4(const Vec<T,4>& v);

    // Cast Operation
    template <class U> Vec4(const Vec<U,4>& v);    
    
    // Mutators
    void set(const T x0, const T x1, const T x2, const T x3);
  };

  // Declare a few common typdefs
  typedef Vec4<bool> Vec4b;
  typedef Vec4<char> Vec4c;
  typedef Vec4<unsigned char> Vec4uc;
  typedef Vec4<int> Vec4i;
  typedef Vec4<float> Vec4f;
  typedef Vec4<double> Vec4d;


  ////////////////////////////////////////////////////////////////////////////
  // DEFINITIONS
  ////////////////////////////////////////////////////////////////////////////
  
  //////////////////// Vec4 ////////////////////
  
  // Constructors 
  
  /** Ctor that doesn't initialize.
   */
  template <class T> 
  inline Vec4<T>::Vec4() {}
  
  /** Ctor that intalizes from array.
   */
  template <class T> 
  inline Vec4<T>::Vec4(const T* d) : Vec<T,4>(d) {}
  
  /** Ctor that intalizes all elements from a scalar.
   */
  template <class T> 
  inline Vec4<T>::Vec4(const T a) : Vec<T,4>(a) {}
  
  /** Ctor that initializes vector with given values.
   */
  template <class T> 
  inline Vec4<T>::Vec4(const T x0, const T x1, 
		       const T x2, const T x3) {
    set(x0,x1,x2,x3);
  }
  
  /** Ctor that initializes an Vec4<T> with an Vec<T,4>.
   */
  template <class T> 
  inline Vec4<T>::Vec4(const Vec<T,4>& v) : Vec<T,4>(v) {}
  
  // Casting Operation

  /** Casting Ctor that initializes an Vec4<T> with a Vec<U,4>.
   */
  template <class T> template <class U>
  inline Vec4<T>::Vec4(const Vec<U,4>& v) : Vec<T,4>(v) {}
  
  // Mutators
  
  /** Set vector.
   */
  template <class T> 
  inline void Vec4<T>::set(const T x0, const T x1, 
			   const T x2, const T x3) {
    x[0] = x0; x[1] = x1; x[2] = x2; x[3] = x3;
  }
} // end namespace sla

#endif
