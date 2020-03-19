//-*-c++-*-
#ifndef SMAT3_H
#define SMAT3_H

/** 
 * @file mat.h
 * @brief Basic static square 3x3 matric type, templated in type and size
 * @author James R. Diebel, Stanford University
 *  
 * - History:
 *  - 31 July 2005 - Started (JD)
 *  - 30 Aug 2005 - Commented and tested (KH)
 *  - 30 Aug 2005 - forked off from matmath.h
 */

#include <mathutil.h>
#include <smat.h>

/**
 * @namespace sla
 * @brief Simple Linear Algebra - library of matrix and vector classes
 */
namespace sla {

  /////////////////////////////////////////////////////////////////////////////
  // DECLARATIONS
  /////////////////////////////////////////////////////////////////////////////

  // Forward declarations
  template <class T, int M> class SMat; // MxM Square Matrix
  template <class T> class Vec3; // 3d Vector
  template <class T> class SMat3; // 3x3 Matrix


  //////////////////// SMat3 //////////////////// 
  /**
   * A 3x3 matrix.
   * A specialization of a square matrix.
   */
  template <class T> 
  class SMat3: public SMat<T,3> {
  public:
    // Data
    using SMat<T,3>::x;
    using SMat<T,3>::set;

    // Constructors
    SMat3();
    SMat3(const T* d);
    SMat3(const T diagVal);
    SMat3(const Vec3<T>& diagVec);
    SMat3(const Mat<T,3,3>& m);
    SMat3(const T x11, const T x12, const T x13,
	  const T x21, const T x22, const T x23,
	  const T x31, const T x32, const T x33);
    SMat3(const Vec3<T>& q0,const Vec3<T>& q1, const Vec3<T>& q2);
      
    
    // Casting Operation
    template <class U> SMat3(const Mat<U,3,3>& m);
    
    // Named Constructors
    static SMat3<T> fromRows(const Vec3<T>& v0, const Vec3<T>& v1,
			     const Vec3<T>& v2);
    static SMat3<T> fromCols(const Vec3<T>& v0, const Vec3<T>& v1,
			     const Vec3<T>& v2);

    // Mutators
    void set(const T x11, const T x12, const T x13,
	     const T x21, const T x22, const T x23,
	     const T x31, const T x32, const T x33);
    void setRows(const Vec3<T>& v0, const Vec3<T>& v1, const Vec3<T>& v2);
    void setCols(const Vec3<T>& v0, const Vec3<T>& v1, const Vec3<T>& v2);

    // Determinant, inverse, etc.
    T det() const;
    SMat3<T> inverted() const;  
    int invert();
  };

  // Declare a few common typdefs
  typedef SMat3<bool> SMat3b;
  typedef SMat3<char> SMat3c;
  typedef SMat3<unsigned char> SMat3uc;
  typedef SMat3<int> SMat3i;
  typedef SMat3<float> SMat3f;
  typedef SMat3<double> SMat3d;


  ////////////////////////////////////////////////////////////////////////////
  // DEFINITIONS
  ////////////////////////////////////////////////////////////////////////////

  //////////////////// SMat3 ////////////////////
  
  // Constructors
  
  /** Ctor that doesn't initialize anything.
   */
  template <class T> 
  inline SMat3<T>::SMat3() {}
  
  /** Ctor that initializes from an array.
   * @param d the (row major) data array of length 4
   */
  template <class T> 
  inline SMat3<T>::SMat3(const T* d) : SMat<T,3>(d) {}
  
  /** Ctor that makes a multiple of the identity matrix.
   * @param diagVal the value to which all diagonal entries will be set
   */
  template <class T> 
  inline SMat3<T>::SMat3(const T diagVal) : SMat<T,3>(diagVal)  {}
  
  /** Ctor that makes a (mostly) zero matrix with diagonal entries from vec.
   * @param diagVec the vector of values that should appear on the diagonal
   */
  template <class T> 
  inline SMat3<T>::SMat3(const Vec3<T>& diagVec) : SMat<T,3>(diagVec) { }
  
  /** Ctor that initializes from a Mat<T,3,3>.
   */
  template <class T> 
  inline SMat3<T>::SMat3(const Mat<T,3,3>& m) : SMat<T,3>(m) {}
  
  /** Ctor that initializes matrix entries directly.
   */
  template <class T> 
  inline SMat3<T>::SMat3(const T x11, const T x12, const T x13,
			 const T x21, const T x22, const T x23,
			 const T x31, const T x32, const T x33) {
    set(x11, x12, x13, x21, x22, x23, x31, x32, x33);
  }
  
  /** Ctor that initializes matrix from columns.
   */
  template <class T> 
  inline SMat3<T>::SMat3(const Vec3<T>& v0, const Vec3<T>& v1, 
			 const Vec3<T>& v2) {
    setCols(v0,v1,v2);
  }
  
  // Casting Operation
  
  /** Casting Ctor that initializes from a Mat<U,3,3> with type cast
   */
  template <class T> template <class U> 
  inline SMat3<T>::SMat3(const Mat<U,3,3>& m) : SMat<T,3>(m) {}

  // Named Constructors

  /** Create matrix from rows.
   */
  template <class T> 
  inline SMat3<T> SMat3<T>::fromRows(const Vec3<T>& v0, const Vec3<T>& v1, 
				     const Vec3<T>& v2) {
    SMat3<T> m;
    m.setRows(v0,v1,v2);
    return m;
  }
  
  /** Create matrix from columns.
   */
  template <class T> 
  inline SMat3<T> SMat3<T>::fromCols(const Vec3<T>& v0, const Vec3<T>& v1, 
				     const Vec3<T>& v2) {
    SMat3<T> m;
    m.setCols(v0,v1,v2);
    return m;
  } 

  // Mutators
  
  /** Set matrix given entries directly.
   */
  template <class T> 
  inline void SMat3<T>::set(const T x11, const T x12, const T x13,
			    const T x21, const T x22, const T x23,
			    const T x31, const T x32, const T x33) {
    x[0] = x11; x[3] = x12; x[6] = x13;
    x[1] = x21; x[4] = x22; x[7] = x23;
    x[2] = x31; x[5] = x32; x[8] = x33;
  }
  
  /** Set the rows of the matrix.
   */
  template <class T> 
  inline void SMat3<T>::setRows(const Vec3<T>& v0, const Vec3<T>& v1, 
				const Vec3<T>& v2) {
    setRow(0,v0); setRow(1,v1); setRow(2,v2);
  }
  
  /** Set the columns of the matrix.
   */
  template <class T> 
  inline void SMat3<T>::setCols(const Vec3<T>& v0, const Vec3<T>& v1, 
				const Vec3<T>& v2) {
    setCol(0,v0); setCol(1,v1); setCol(2,v2);
  }

  // Determinant and Inverse
  
  /** Calculate determinant.
   */
  template <class T> 
  inline  T SMat3<T>::det() const {
    return (x[0]*(x[4]*x[8] - x[5]*x[7]) -
	          x[3]*(x[1]*x[8] - x[2]*x[7]) +
	          x[6]*(x[1]*x[5] - x[2]*x[4]));
  }
  
 /** Invert the matrix. (matrix set to its inverse)
   * @todo consider throwing exception instead
   */
  template <class T>
  inline SMat3<T> SMat3<T>::inverted() const {
    SMat3<T> tmp(*this); 
    tmp.invert(); 
    return tmp;
  }
  
  /** Perform inverse in place
   */
  template <class T>
  inline int SMat3<T>::invert() {
    T d = det();
    if (d == 0.0) {
      std::cerr << "SMat3::invert(): warning, "
		<< "can't take inverse of singular matrix." << std::endl << std::flush;
      return 1;
    }
    T di = T(1)/d;
    set((x[4]*x[8] - x[7]*x[5])*di, (x[5]*x[6] - x[3]*x[8])*di, (x[3]*x[7] - x[4]*x[6])*di, 
        (x[2]*x[7] - x[1]*x[8])*di, (x[0]*x[8] - x[6]*x[2])*di, (x[1]*x[6] - x[0]*x[7])*di,
	      (x[1]*x[5] - x[2]*x[4])*di, (x[2]*x[3] - x[0]*x[5])*di, (x[0]*x[4] - x[3]*x[1])*di);
  }
} // end namespace sla

#endif
