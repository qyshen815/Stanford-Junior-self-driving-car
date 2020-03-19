//-*-c++-*-
#ifndef FLEXEDGE_H
#define FLEXEDGE_H

/*
  James R. Diebel
  Stanford University
  
  Started: 29 November 2005

  General tmeplated triangular mesh edge class, part of FlexMesh 
  class structure

  Template parameters:
  - floating point type T
*/

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>

#define FE_CHECK_BOUNDS 1
#define FE_PO 2

// Forward Declarations
template <class T> class FlexVertex;
template <class T> class FlexFace;
template <class T> class FlexMesh;

////////////////////
// FlexEdge Class //
////////////////////

template <class T>
class FlexEdge {
public:
  // Constructor/Destructor
  FlexEdge();

  // Public Data
  sla::Vec3<T> x; // position of mid-point of edge
  T w; // weight based upon color gradient

  // Public Methods
  FlexVertex<T>& v(const int i);
  FlexVertex<T>& v(const int i) const;
  FlexFace<T>& f(const int i);
  FlexFace<T>& f(const int i) const;
  int numf() const;
  void writeTo(std::ostream& os); 
  void readFrom(std::istream& is); 
  void print(std::ostream& os) const;

  // Find Derived Data
  void findCenter();
  
protected:
  // Protected Data
  int nf; // numbers of neighboring faces
  FlexVertex<T>* vd[2]; // pointers to vertices that make up this face
  FlexFace<T>* fd[2]; // pointers to neighboring faces

  friend class FlexMesh<T>;
};

// Instantiate a few instances
template class FlexEdge<float>;
template class FlexEdge<double>;

////////////////////////
// FlexEdge Methods //
////////////////////////

// Constructor
template <class T> 
inline FlexEdge<T>::FlexEdge() {
  nf = 0;
  vd[0] = vd[1] = NULL;
  fd[0] = fd[1] = NULL;
  w = T(0.0);
}

// Returns reference to vertex i of this edge
template <class T> 
inline FlexVertex<T>& FlexEdge<T>::v(const int i) {
#if FE_CHECK_BOUNDS
  if (i < 0 || i >= 2) {
    printf("Vertex index %i out of range [0,%i], exiting.\n", i, 2);
    std::exit(1);
  }
#endif
  return *(vd[i]);
}

// Returns const reference to vertex i of this edge
template <class T> 
inline FlexVertex<T>& FlexEdge<T>::v(const int i) const {
#if FE_CHECK_BOUNDS
  if (i < 0 || i >= 2) {
    printf("Vertex index %i out of range [0,%i], exiting.\n", i, 2);
    std::exit(1);
  }
#endif
  return *(vd[i]);
}

// Returns reference to neighboring face i of this vertex
template <class T> 
inline FlexFace<T>& FlexEdge<T>::f(const int i) {
#if FE_CHECK_BOUNDS
  if (i < 0 || i >= nf) {
    printf("Face index %i out of range [0,%i], exiting.\n", i, nf);
    std::exit(1);
  }
#endif
  return *(fd[i]);
}

// Returns const reference to neighboring face i of this vertex
template <class T> 
inline FlexFace<T>& FlexEdge<T>::f(const int i) const {
#if FE_CHECK_BOUNDS
  if (i < 0 || i >= nf) {
    printf("Face index %i out of range [0,%i], exiting.\n", i, nf);
    std::exit(1);
  }
#endif
  return *(fd[i]);
}

// Returns the total number of faces
template <class T> 
inline int FlexEdge<T>::numf() const {
  return nf;
}


// Find Edge center as average of vertex positions
template <class T> 
inline void FlexEdge<T>::findCenter() {
  x = T(0.5)*(v(0).x + v(1).x);
}

#endif
