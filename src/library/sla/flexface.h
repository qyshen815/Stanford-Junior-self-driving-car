//-*-c++-*-
#ifndef FLEXFACE_H
#define FLEXFACE_H

/*
  James R. Diebel
  Stanford University
  
  Started: 29 November 2005

  General templated triangular mesh face class, part of FlexMesh 
  class structure

  Template parameters:
  - floating point type T
*/

#include "vec3.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>

#define FF_CHECK_BOUNDS 1
#define FF_PO 2

// Forward Declarations
template <class T> class FlexVertex;
template <class T> class FlexFace;
template <class T> class FlexEdge;
template <class T> class FlexMesh;

////////////////////
// FlexFace Class //
////////////////////

template <class T> 
class FlexFace {
public:
  // Public Data
  sla::Vec3<T> n; // normal vector of face
  sla::Vec3<T> n0; // reference normal vector
  sla::Vec3<T> x; // center of face
  int flag; // general purpose integer face flag

  // Public Methods
  FlexFace();
  FlexVertex<T>& v(const int i);
  FlexVertex<T>& v(const int i) const;
  FlexFace<T>& f(const int i);
  FlexFace<T>& f(const int i) const;
  int numf() const;
  void writeTo(std::ostream& os, FlexVertex<T>* v0); 
  void readFrom(std::istream& is, FlexVertex<T>* v0); 
  void print(std::ostream& os, FlexVertex<T>* v0) const;

  // Find Derived Data
  void findCenter();

protected:
  // Protected Data
  int nf;
  FlexVertex<T>* vd[3]; // pointers to vertices that make up this face
  FlexFace<T>* fd[3]; // pointers to neighboring faces
  FlexEdge<T>* ed[3]; // pointers to neightboring edges
  
  // Protected members
  void setVertexIndices(const sla::Vec3<int>& ind, FlexVertex<T>* v0);
  sla::Vec3<int> getVertexIndices(FlexVertex<T>* v0) const;
  
  friend class FlexMesh<T>;
};

// Instantiate a few instances
template class FlexFace<float>;
template class FlexFace<double>;

////////////////////////////
// FlexFace Methods //
////////////////////////////

// Constructor
template <class T> 
inline FlexFace<T>::FlexFace() {
  nf = 0;
}

// Returns reference to vertex i of this face
template <class T> 
inline FlexVertex<T>& FlexFace<T>::v(const int i) {
#if FF_CHECK_BOUNDS
  if (i < 0 || i >= 3) {
    printf("Vertex index %i out of range [0,%i], exiting.\n", i, 3);
    std::exit(1);
  }
#endif
  return *(vd[i]);
}

// Returns const reference to vertex i of this face
template <class T> 
inline FlexVertex<T>& FlexFace<T>::v(const int i) const {
#if FF_CHECK_BOUNDS
  if (i < 0 || i >= 3) {
    printf("Vertex index %i out of range [0,%i], exiting.\n", i, 3);
    std::exit(1);
  }
#endif
  return *(vd[i]);
}

// Returns reference to neighboring face i of this face
template <class T> 
inline FlexFace<T>& FlexFace<T>::f(const int i) {
#if FF_CHECK_BOUNDS
  if (i < 0 || i >= nf) {
    printf("Face: Face index %i out of range [0,%i], exiting.\n", i, nf);
    std::exit(1);
  }
#endif
  return *(fd[i]);
}

// Returns const reference to neighboring face i of this face
template <class T> 
inline FlexFace<T>& FlexFace<T>::f(const int i) const {
#if FF_CHECK_BOUNDS
  if (i < 0 || i >= nf) {
    printf("Face: Face index %i out of range [0,%i], exiting.\n", i, nf);
    std::exit(1);
  }
#endif
  return *(fd[i]);
}

// Returns the number of neighboring faces
template <class T> 
inline int FlexFace<T>::numf() const {
  return nf;
}

// Sets vertex i of this face to point to the given vertex
template <class T> 
inline void FlexFace<T>::setVertexIndices(const sla::Vec3<int>& ind,
					  FlexVertex<T>* v) { 
  for (int i=0;i<3;i++) vd[i] = &(v[ind(i)]);
}

// Returns the indices of the vertices of this face relative to given ref
template <class T>
inline sla::Vec3<int> FlexFace<T>::getVertexIndices(FlexVertex<T>* v0) const {
  return sla::Vec3<int>(int(vd[0] - v0),int(vd[1] - v0),int(vd[2] - v0));
}

// Writes state to stream (binary)
template <class T> 
inline void FlexFace<T>::writeTo(std::ostream& os, 
				 FlexVertex<T>* v0) {
  sla::Vec3i ind(getVertexIndices(v0));
  ind.writeTo(os);
  n.writeTo(os);
}

// Restores state from stream (binary)
template <class T> 
inline void FlexFace<T>::readFrom(std::istream& is, 
				  FlexVertex<T>* v0) {
  sla::Vec3i ind;
  ind.readFrom(is);
  setVertexIndices(ind, v0);
  n.readFrom(is);
}

// Writes state as formated ascii to given stream
template <class T> 
inline void FlexFace<T>::print(std::ostream& os, 
			       FlexVertex<T>* v0) const {
  sla::Vec3i ind(getVertexIndices(v0));
  os << "indices: " << ind;
  os << "normal: " << n;
}

// Find Face center as average of vertex positions
template <class T> 
inline void FlexFace<T>::findCenter() {
  x.set(T(0));
  for (int i=0;i<3;i++) {
    x += v(i).x;
  }
  x /= T(3);
}

#endif
