//-*-c++-*-
#ifndef ARRAY_H
#define ARRAY_H

/** 
 * @file array.h
 * @brief Basic dynamic K-dimensional array of type T - COLUMNWISE ORDERING!
 * @author James R. Diebel, Stanford University
 *  
 * - History:
 *  - 3 March 2006 - Started 
 */

#include <matmath.h>

#ifdef _DEBUG
 #define AR_CHECK_BOUNDS 1
 #define AR_PO 1
#else
 #define AR_CHECK_BOUNDS 0
 #define AR_PO 0
#endif

/**
 * @namespace sla
 * @brief Simple Linear Algebra - library of matrix and vector classes 
 * plus some random utility functions and other containers
 */
namespace sla {
  
  /////////////////////////////////////////////////////////////////////////////
  // DECLARATIONS
  /////////////////////////////////////////////////////////////////////////////
  
  // Forward declarations	
  template <class T, int K> class Array; // Arbitrary-dimensional array

  //////////////////// Array ////////////////////
  /**
   * A K-Dimensional Array of type T
   */
  template <class T, int K> 
  class Array {
  public:
    // Constructors/Destructor
    Array();
    Array(const Vec<int,K> dim_);
    Array(const Array<T,K>& a) { set(a); }
    ~Array();
    
    // Mutators
    void deleteArray() { reset(); }
    void reset();
    void setSize(const Vec<int,K> dim_);
    void set(const T v);
    void set(const T* v, const int i0 = 0, const int vlen = -1);
    void set(const Array<T,K>& a);
    void addTo(const T* v, const int i0 = 0, const int vlen = -1);
    T& operator () (const int i);
    T& operator () (const Vec<int,K> ind);
    
    // Assignment operator
    Array<T,K>& operator = (const Array<T,K>& a) { set(a); return *this; }

    // Accessors
    Vec<int,K> size() const;
    int size(const int i) const;
    int length() const;
    T& operator () (const int i) const;
    T& operator () (const Vec<int,K> ind) const;
    T min() const;
    T max() const;

    // Serialization
    int writeTo(std::ostream& os) const;
    int readFrom(std::istream& is);

    // Helper functions
    int indexOf(Vec<int,K> ind) const;
    Vec<int,K> coordsOf(int ind) const;
    bool isInBounds(Vec<int,K> ind) const;
    bool isInBounds(int ind) const;
    int find(const T element) const; 
    bool contains(const T element) const; 
    
    // Data
    T* x; ///< storage array
  protected:
    Vec<int,K> dim; ///< array dimensions
  };

  // ASCII stream IO
  template <class T, int K>  
  std::ostream& operator << (std::ostream& os, const Array<T,K>& a);
  template <class T, int K> 
  std::istream& operator >> (std::istream& is, Array<T,K>& a);
  
  ////////////////////////////////////////////////////////////////////////////
  // DEFINITIONS
  ////////////////////////////////////////////////////////////////////////////
     
  //////////////////// Array ////////////////////

  // Constructors/Destructor
  
  /** Ctor that does no initalization.
   */
  template <class T, int K> 
  inline Array<T,K>::Array() {
    x = NULL;
    dim.set((int)0);
    if (AR_PO>1) std::cout << "Created empty Array." 
			   << std::endl << std::flush;
  }
  
  /** Ctor that starts with given dimensions
   * @param Vec<int,K> n_ are the sizes in the several dimension
   */
  template <class T, int K> 
  inline Array<T,K>::Array(const Vec<int,K> dim_) : Array<T,K>() {
    setSize(dim_);
  }
  
  /** Dtor.
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
    dim.set(0);
    if (x) {
      delete [] x;
      if (AR_PO>1) std::cout << "Freed memory for array." 
			     << std::endl << std::flush;
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
      for (int k=0;k<K;k++) if (dim_(k) <= 0) {
        if (AR_PO>1) {
          std::cout << dim_;
          std::cout << "Warning, negative or zero dimensions not allowed." 
            << std::endl << std::flush;
        }
      }
      dim = dim_;
      int len = dim.prod();
      if (len>0) {
        x = new T[len];
        if (AR_PO>1) std::cout << "Created array of linear size " << len 
          << "." << std::endl << std::flush;
      }
    }
  }
  
  /** Set the values of all the elements in the array to v
   * @param v is the value to set the array to
   */
  template <class T, int K> 
  inline void Array<T,K>::set(const T v) {
    int len = dim.prod();
    for (int i=0;i<len;i++) x[i] = v;
    if (AR_PO>3) std::cout << "Set array elements to passed value."
			   << std::endl << std::flush;
  }

  /** Set the values of all the elements in the array to v
   * @param v is the value to set the array to
   */
  /*template <int K>
  inline void Array<,K>::set(const T v) {
    int len = dim.prod();
    if (v  == T(0)) memset(x,0,len*sizeof(T));
    for (int i=0;i<len;i++) x[i] = v;
    if (AR_PO>3) std::cout << "Set array elements to passed value."
			   << std::endl << std::flush;
  }*/

  /** Set the values of all the elements in the array to v
   * @param v is the value to set the array to
   */
  template <class T, int K> 
  inline void Array<T,K>::set(const T* v, const int i0, const int vlen) {
    int len = dim.prod();
    int i1 = (vlen == -1)? len:sla::clamp(i0+vlen,0,len);
    for (int i=i0,j=0;i<i1;i++,j++) x[i] = v[j];
    if (AR_PO>3) std::cout << "Set array elements to passed values."
			   << std::endl << std::flush;
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
   */
  template <class T, int K> 
  inline void Array<T,K>::addTo(const T* v, const int i0, const int vlen) {
    int len = dim.prod();
    int i1 = (vlen == -1)? len:sla::clamp(i0+vlen,0,len);
    for (int i=i0,j=0;i<i1;i++,j++) x[i] += v[j];
    if (AR_PO>3) std::cout << "Added passed values to array elements."
			   << std::endl << std::flush;
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
  inline T& Array<T,K>::operator () (const int i) {
#if AR_CHECK_BOUNDS
    int len = dim.prod();
    if (i<0 || i>=len) {
      std::cerr << "Array<T," << K << ">::operator (): Error. Index " 
		<< i << " exceeds bounds [0, "
		<< len << "]." << std::endl << std::flush;
      exit(1);
    }
#endif
    return x[i];
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
  inline int Array<T,K>::size(const int i) const {
    return dim(i);
  }

  /** Returns linear length of data
      @return integer legth of data
   */
  template <class T, int K> 
  inline int Array<T,K>::length() const {
    return dim.prod();
  }

  /** Returns inmutable reference to array element
   * @param ind is the index list
   * @return the value of the array element
   */
  template <class T, int K>
  inline T& Array<T,K>::operator () (const Vec<int,K> ind) const {
    return x[indexOf(ind)];
  }

  /** Returns inmutable reference to array element
   * @param i is the linear index
   * @return the value of the array element
   */
  template <class T, int K> 
  inline T& Array<T,K>::operator () (const int i) const {
#if AR_CHECK_BOUNDS
    int len = dim.prod();
    if (i<0 || i>=len) {
      std::cerr << "Error. Index " << i << " exceeds bounds [0, "
		<< len << "]." << std::endl << std::flush;
      exit(1);
    }
#endif
    return x[i];
  }

  /** Returns the minimum value in the array.
   */
  template <class T, int K> 
  inline T Array<T,K>::min() const {
    if (length() == 0) {
      std::cerr << "Array<T,K>::min: Error, no elements in array. Exiting."
		<< std::endl << std::flush;
    }
    int len = dim.prod();
    T m = x[0];
    for (int i=0;i<len;i++) {
      if (x[i] < m) m = x[i];
    }
    return m;
  }

  /** Returns the maximum value in the array.
   */
  template <class T, int K> 
  inline T Array<T,K>::max() const {
    if (length() == 0) {
      std::cerr << "Array<T,K>::max: Error, no elements in array. Exiting."
		<< std::endl << std::flush;
    }
    int len = dim.prod();
    T m = x[0];
    for (int i=0;i<len;i++) {
      if (x[i] > m) m = x[i];
    }
    return m;    
  }


  // Serialization routines
  
  /** Write state to a binary stream.
   */
  template <class T, int K> 
  inline int Array<T,K>::writeTo(std::ostream& os) const {
    char flag[6] = "Array";
    os.write(flag,5*sizeof(char));
    dim.writeTo(os);
    int len = dim.prod();
    if (len>0) os.write((char *)(x),len*sizeof(T));
    return 0;
  }
  
  /** Restore state from a binary stream.
   */
  template <class T, int K> 
  inline int Array<T,K>::readFrom(std::istream& is) {
    char flag[6];
    is.read(flag,5*sizeof(char));
    if (flag[0] != 'A' || flag[1] != 'r') {
      if (AR_PO>1) 
	std::cout << "Expecting array flag, but got something else." 
		  << std::endl << std::flush;
      return 1;
    }
    Vec<int,K> dim_;
    dim_.readFrom(is);
    if (!dim_.equalTo(Array<T,K>::size())) Array<T,K>::setSize(dim_);    
    int len = dim.prod();
    if (len>0) is.read((char *)(x),len*sizeof(T));
    return 0;
  }

  /** Write state to stream as formated ASCII 
   */
  template <class T, int K> 
  inline std::ostream& operator << (std::ostream& os, const Array<T,K>& a) {
    int minFieldWidth = os.precision()+5;  
    Vec<int,K> dim = a.size();
    for (int i=0;i<K;i++) os << dim(i) << " ";
    os << std::endl;
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
  inline std::istream& operator >> (std::istream& is, Array<T,K>& a) {
    Vec<int,K> dim;
    for (int i=0;i<K;i++) is >> dim(i);
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
      std::cerr << "Error. Index " << ind(k) << " exceeds bounds [0, " 
		<< dim(k) << "]"
		<< std::endl << std::flush;
      exit(1);
    }
#endif
    int k=ind(0);
    int mul = 1;
    for (int i=1;i<K;i++) {
      k += mul*ind(i);
      mul *= dim(i-1);
    }
    return k;
  }

  /** Returns component indices for the linear array index
   * @param ind is the linear array index to be converted
   * @return the component indices (coords) in the data array x
   */
  template <class T, int K> 
  inline Vec<int,K> Array<T,K>::coordsOf(int ind) const {
#if AR_CHECK_BOUNDS
    if (ind<0 || ind>=length()) {
      std::cerr << "Error. Index " << ind << " exceeds bounds [0, " 
		<< length() << "]"
		<< std::endl << std::flush;
      exit(1);
    }
#endif
    Vec<int,K> result; 
    for (int id=0;id<K-1;id++) {
      result(id) = ind % dim(id); 
      ind /= dim(id); 
    }
    result(K-1) = ind; 
    return result;
  }

  /** Checks whether the given indices are within bounds
   * @param ind is the vector of indices to be checked
   * @return a bool: true if indices are in bounds, false if out of bounds
   */
  template <class T, int K> 
  inline bool Array<T,K>::isInBounds(Vec<int,K> ind) const {
    for (int k=0;k<K;k++) if (ind(k) < 0 || ind(k) >= dim(k)) return false;
    return true;
  }

  /** Checks if the given index is within array bounds
  * @param ind is the index to check
  */
  template <class T, int K> 
  inline bool Array<T,K>::isInBounds(int ind) const {
    return (ind>=0 && ind<length());
  }


  /** Finds the index of an element in the array. 
  * @param element is the element to find
  * @return an int: the index of the element. -1 if it could not be found. 
  */
  template <class T, int K> 
  inline int Array<T,K>::find(const T element) const {
    for (int k=0,n=length();k<n;k++)
      if (x[k] == element)
        return k;
    return -1; 
  }

  /** Checks if the given element is in this array
  * @param element is the element to check
  * @return a bool: true if the element is in the array, false if not
  */
  template <class T, int K> 
  inline bool Array<T,K>::contains(const T element) const {
    return find(element) >= 0;
  }


} // end namespace sla
#endif  
