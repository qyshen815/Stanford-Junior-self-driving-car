//-*-c++-*-
#ifndef FLEXMESH_H
#define FLEXMESH_H

/*
  James R. Diebel
  Stanford University
  
  Started: 29 November 2005

  General templated triangular mesh class capable of:
  - finding face-vertex-edge connectivity from vertex/face lists
  - finding per-face and per-vertex normal vectors

  Depends on FlexVertex, FlexFace, and FlexEdge classes, which are friends.

  Template parameters:
  - floating point type T
*/

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <list>

#include "matmath.h"
#include "flexvertex.h"
#include "flexface.h"
#include "flexedge.h"

#define FM_CHECK_BOUNDS 1
#define FM_PO 2

// Forward Declarations
template <class T> class FlexVertex;
template <class T> class FlexFace;
template <class T> class FlexEdge;
template <class T> class FlexMesh;

//////////////////////
// FlexMesh Class //
//////////////////////

template <class T>
class FlexMesh {
public:
  // Constructors/Destructor and Related
  FlexMesh();
  FlexMesh(const FlexMesh<T>& m);
  FlexMesh(int nv_, int nf_);
  ~FlexMesh();

  // Memory Allocation and Deallocation
  void startUp(int nv_, int nf_); // allocate memory
  void cleanUp(); // deallocate memory

  // References
  FlexVertex<T>& v(const int i);
  FlexVertex<T>& v(const int i) const;
  FlexEdge<T>& e(const int i);
  FlexEdge<T>& e(const int i) const;
  FlexFace<T>& f(const int i);
  FlexFace<T>& f(const int i) const;
  int numv() const;
  int nume() const;
  int numf() const;
  sla::Vec3<int> getFace(const int i) const;

  // Assignments
  void set(const FlexMesh<T>& m);
  FlexMesh<T>& operator = (const FlexMesh<T>& m);
  void setVertex(const int i, bool boundary_,sla::Vec3<T> x_,
  sla::Vec3<unsigned char> c_);
  void setFace(const int i, const sla::Vec3<int>& ind);

  // File I/O
  int readBSM(const char* filename); // read Binary FlexMesh
  int writeBSM(char* filename); // write Binary FlexMesh
  void writeTo(std::ostream& os); 
  void readFrom(std::istream& is); 
  void print(std::ostream& os) const; 
  void print();

  // Find Derived Data
  void findFaceNormals();
  void findVertexNormals();
  void findNormals();

  void findFaceCenters();
  void findEdgeCenters();

  void findVertexToFace();
  void findVertexToVertex();
  void findFaceToFace();
  void findEdges();

  // Methods relating to Color-based smoothing
  void storeReferenceData();
  void findColorWeights();
  void findGradient(); // compute gradient of potential function
  void takeHillClimbingStep(); // take hill-climbing step

protected:
  // Protected Data 
  int nv, nf, ne; // numbers of vertices, faces, edges
  int nn; // total number of neighboring triangles
  FlexVertex<T>* vd; // global vertex list
  FlexFace<T>* fd; // global face list
  FlexEdge<T>* ed; // global edge list
  bool haveVertexToFace;
  bool haveVertexToVertex;
  bool haveFaceToFace;
  bool haveEdges;
};

// Instantiate a few instances
template class FlexMesh<float>;
template class FlexMesh<double>;

// Declare a few common typdefs
typedef FlexMesh<float> FlexMeshf;
typedef FlexMesh<double> FlexMeshd;

////////////////////////
// FlexMesh Methods //
////////////////////////

// Constructor: default
template <class T> 
inline FlexMesh<T>::FlexMesh() {
  nv = nf = ne = nn = 0;
  haveVertexToFace = false;
  haveVertexToVertex = false;
  haveFaceToFace = false;
  haveEdges = false;
  vd = NULL; fd = NULL;
}

// Constructor: copy
template <class T> 
inline FlexMesh<T>::FlexMesh(const FlexMesh<T>& m) {
  this->set(m);
}

// Constructor: with given size
template <class T> 
inline FlexMesh<T>::FlexMesh(int nv_, int nf_) {
  nv = nf = ne = nn = 0;
  haveVertexToFace = false;
  haveVertexToVertex = false;
  haveFaceToFace = false;
  haveEdges = false;
  vd = NULL; fd = NULL;
  startUp(nv_,nf_);
}

// Destructor: cleans up allocated memory
template <class T> 
inline FlexMesh<T>::~FlexMesh() {
  cleanUp();
}

// Memory allocation
template <class T> 
inline void FlexMesh<T>::startUp(int nv_, int nf_) {
  cleanUp(); // always clean up first, in case of old data
  nv = nv_; nf = nf_;
  if (FM_PO>1) printf("Allocating memory for %i vertices and %i faces...",
		      nv,nf);
  vd = new FlexVertex<T>[nv];
  fd = new FlexFace<T>[nf];
  if (FM_PO>1) printf("Done.\n");
}

// Memory deallocation
template <class T> 
inline void FlexMesh<T>::cleanUp() {
  if (nv != 0) {
    if (FM_PO>1) printf("Deleting vertex list (%i)...",nv);
    delete [] vd;
    if (FM_PO>1) printf("Done.\n");
  }
  if (nf != 0) {
    if (FM_PO>1) printf("Deleting face list (%i)...", nf);
    delete [] fd;
    if (FM_PO>1) printf("Done.\n");
  }
  nv = nf = ne = nn = 0;
  haveVertexToFace = false;
  haveVertexToVertex = false;
  haveFaceToFace = false;
  haveEdges = false;
  vd = NULL; fd = NULL;
}

// Assignment operator
template <class T> 
FlexMesh<T>& FlexMesh<T>::operator = (const FlexMesh<T>& m) {
  this->set(m);
  return *this;
}

// Assignment function
template <class T> 
void FlexMesh<T>::set(const FlexMesh<T>& m) {
  nv = nf = ne = nn = 0;
  vd = NULL; fd = NULL;
  startUp(m.nv,m.nf);
  for (int i=0;i<nv;i++) {
    v(i).boundary = m.v(i).boundary;
    v(i).x = m.v(i).x;
    v(i).c = m.v(i).c;
    v(i).n = m.v(i).n;
  }
  for (int i=0;i<nf;i++) {
    sla::Vec3i ind(m.getFace(i));
    f(i).setVertexIndices(ind, vd);
    f(i).n = m.f(i).n;
  }
}


// Returns reference to vertex i in the global vertex list
template <class T> 
inline FlexVertex<T>& FlexMesh<T>::v(const int i) {
#if FM_CHECK_BOUNDS
  if (i < 0 || i >= nv) {
    printf("Vertex index %i out of range [0,%i], exiting.\n", i, nv);
    std::exit(1);
  }
#endif
  return vd[i];
}

// Returns vertex i in the global vertex list
template <class T> 
inline FlexVertex<T>& FlexMesh<T>::v(const int i) const {
#if FM_CHECK_BOUNDS
  if (i < 0 || i >= nv) {
    printf("Vertex index %i out of range [0,%i], exiting.\n", i, nv);
    std::exit(1);
  }
#endif
  return vd[i];
}

// Returns reference to edge i in the global edge list
template <class T> 
inline FlexEdge<T>& FlexMesh<T>::e(const int i) {
#if FM_CHECK_BOUNDS
  if (i < 0 || i >= ne) {
    printf("Edge index %i out of range [0,%i], exiting.\n", i, ne);
    std::exit(1);
  }
#endif
  return ed[i];
}

// Returns edge i in the global edge list
template <class T> 
inline FlexEdge<T>& FlexMesh<T>::e(const int i) const {
#if FM_CHECK_BOUNDS
  if (i < 0 || i >= ne) {
    printf("Edge index %i out of range [0,%i], exiting.\n", i, ne);
    std::exit(1);
  }
#endif
  return ed[i];
}

// Returns reference to face i in the global face list
template <class T> 
inline FlexFace<T>& FlexMesh<T>::f(const int i) {
#if FM_CHECK_BOUNDS
  if (i < 0 || i >= nf) {
    printf("Face index %i out of range [0,%i], exiting.\n", i, nf);
    std::exit(1);
  }
#endif
  return fd[i];
}

// Returns face i in the global face list
template <class T> 
inline FlexFace<T>& FlexMesh<T>::f(const int i) const {
#if FM_CHECK_BOUNDS
  if (i < 0 || i >= nf) {
    printf("Face index %i out of range [0,%i], exiting.\n", i, nf);
    std::exit(1);
  }
#endif
  return fd[i];
}

// Returns the total number of vertices
template <class T> 
inline int FlexMesh<T>::numv() const {
  return nv;
}

// Returns the total number of edges
template <class T> 
inline int FlexMesh<T>::nume() const {
  return ne;
}

// Returns the total number of faces
template <class T> 
inline int FlexMesh<T>::numf() const {
  return nf;
}

// Get face-vertex indices to passed values
template <class T> 
inline sla::Vec3<int> FlexMesh<T>::getFace(const int i) const {
  return f(i).getVertexIndices(vd);
}


// Set vertex position and color to passed values
template <class T> 
inline void FlexMesh<T>::setVertex(const int i, bool boundary_,
				     sla::Vec3<T> x_, 
				     sla::Vec3<unsigned char> c_) {
  v(i).boundary = boundary_; v(i).x = x_; v(i).c = c_;
}

// Set face-vertex indices to passed values
template <class T> 
inline void FlexMesh<T>::setFace(const int i, const sla::Vec3<int>& ind) {
#if FM_CHECK_BOUNDS
  for (int j=0;j<3;j++) if (ind(j) < 0 || ind(j) >= nv) {
    printf("Vertex index %i out of range [0,%i], exiting.\n", ind(j), nf);
    std::exit(1);
  }
#endif
  f(i).setVertexIndices(ind,vd);
}

// I/O: Writes state to stream (binary)
template <class T> 
inline void FlexMesh<T>::writeTo(std::ostream& os) {
  //write number of vertices and faces to file
  os.write(reinterpret_cast<char *>(&nv),sizeof(int));
  os.write(reinterpret_cast<char *>(&nf),sizeof(int));
 
  //write all vertices to file
  for (int i=0;i<nv;i++) v(i).writeTo(os);
  
  //write all faces to file 
  for (int i=0;i<nf;i++) f(i).writeTo(os,vd);
}

// I/O: Restores state from stream (binary)
template <class T> 
inline void FlexMesh<T>::readFrom(std::istream& is) {
  //read number of vertices and faces from stream
  int nv_, nf_;
  is.read(reinterpret_cast<char *>(&nv_),sizeof(int));
  is.read(reinterpret_cast<char *>(&nf_),sizeof(int));
  
  //allocate memory
  startUp(nv_, nf_);
  
  //read all vertices from stream
  for (int i=0;i<nv;i++) v(i).readFrom(is);
  
  //read all faces from stream 
  for (int i=0;i<nf;i++) f(i).readFrom(is,vd);
}

// I/O: Writes state as formated ascii to given stream
template <class T> 
inline void FlexMesh<T>::print(std::ostream& os) const {
  os << "Not implemented yet!" << std::endl;
}

// I/O: Reads binary file of mesh
template <class T> 
      inline int FlexMesh<T>::readBSM(const char* filename) {
  if (FM_PO>0) printf("Reading BSM from %s ...\n", filename);
  std::fstream fin;
  fin.open(filename,std::ios::in|std::ios::binary);
  if (fin.is_open()) {
    readFrom(fin);
  } else {
    printf("Couldn't open file, exiting...\n");
    std::exit(1);
  }
  fin.close();
  if (FM_PO>0) printf("Done.\n");
  return 0;
}

// I/O: Writes binary file format
template <class T> 
inline int FlexMesh<T>::writeBSM(char* filename) {
  if (FM_PO>0) printf("Writing BSM to %s ...\n", filename);
  std::fstream fout;
  fout.open(filename,std::ios::out|std::ios::binary);
  if (fout.is_open()) {
    writeTo(fout);
  } else {
    printf("Couldn't open file, exiting...\n");
    std::exit(1);
  }
  fout.close();
  if (FM_PO>0) printf("Done.\n");  
  return 0;
}

// Displays ascii mesh to screen
template <class T> 
inline void FlexMesh<T>::print() {
  if (FM_PO>0) printf("Printing ASM to screen...\n");
  // write header
  printf("%i %i\n", nv, nf);
  // write all vertices
  for (int i=0;i<nv;i++) {
    printf("%i %10.5f %10.5f %10.5f %i %i %i\n", 
	   v(i).boundary, 
	   v(i).x(0), v(i).x(1), v(i).x(2),
	   v(i).c(0), v(i).c(1), v(i).c(2));
  }
  // write all faces
  for (int i=0;i<nf;i++) {
    sla::Vec3i ind(getFace(i));
    printf("%i %i %i\n", ind(0), ind(1), ind(2));
  }
  if (FM_PO>0) printf("Done.\n");  
}

// Find face normals
template <class T> 
inline void FlexMesh<T>::findFaceNormals() {
  if (FM_PO>0) printf("Finding face normals...");
  for (int i=0;i<nf;i++) {
    f(i).n = sla::Vec3<T>(f(i).v(1).x - f(i).v(0).x).cross
      (sla::Vec3<T>(f(i).v(2).x - f(i).v(0).x));
    f(i).n.normalize();
  }
  if (FM_PO>0) printf("Done.\n");
}

// Find vertex normals
template <class T> 
inline void FlexMesh<T>::findVertexNormals() {
  if (FM_PO>0) printf("Finding vertex normals...");
  for (int i=0;i<nv;i++) v(i).n = T(0);
  for (int i=0;i<nf;i++) for (int j=0;j<3;j++) f(i).v(j).n += f(i).n;
  for (int i=0;i<nv;i++) v(i).n.normalize();
  if (FM_PO>0) printf("Done.\n");
}

// Find face and vertex normals
template <class T> 
inline void FlexMesh<T>::findNormals() {
  findFaceNormals();
  findVertexNormals();
}

// Find goemetric center of all faces
template <class T> 
inline void FlexMesh<T>::findFaceCenters() {
  for (int i=0;i<nf;i++) f(i).findCenter();
}

// Find goemetric center of all faces
template <class T> 
inline void FlexMesh<T>::findEdgeCenters() {
  for (int i=0;i<ne;i++) e(i).findCenter();
}

// Find vertex-to-face mappings
template <class T> 
inline void FlexMesh<T>::findVertexToFace() {
  if (FM_PO>1) std::cout << "Finding vertex-to-face mappings..." 
			 << std::flush;

  // clean up previously-allocated memory/mappings
  if (haveVertexToFace){
    if (FM_PO>1) std::cout << "deleting old mappings..." << std::flush;
    for (int i=0;i<nv;i++) if (v(i).nf > 0) { 
      delete [] v(i).fd; v(i).nf = 0; v(i).fd = NULL;
    }
    if (FM_PO>1) std::cout << "Done. " << std::flush;
  }
  
  // count the number of faces touching each vertex
  for (int i=0;i<nf;i++) for (int j=0;j<3;j++) f(i).v(j).nf++;
  
  // allocate memory for mappings
  for (int i=0;i<nv;i++) if (v(i).nf > 0) {
    v(i).fd = new FlexFace<T>*[v(i).nf];
    v(i).nf = 0;
  }
  
  // fill mappings
  for (int i=0;i<nf;i++) for (int j=0;j<3;j++) {
    f(i).v(j).fd[f(i).v(j).nf] = &(f(i));
    f(i).v(j).nf++;
  }

  haveVertexToFace = true;

  if (FM_PO>1) std::cout << "Done." << std::endl << std::flush;  
}

// Find vertex-to-vertex mappings
template <class T> 
inline void FlexMesh<T>::findVertexToVertex() {
  if (!haveVertexToFace) findVertexToFace();
  
  if (FM_PO>1) std::cout << "Finding vertex-to-vertex mappings..." 
			 << std::flush;
  
  // clean up previously-allocated memory/mappings
  if (haveVertexToVertex){
    if (FM_PO>1) std::cout << "deleting old mappings..." << std::flush;
    for (int i=0;i<nv;i++) if (v(i).nv > 0) { 
      delete [] v(i).vd; v(i).nv = 0; v(i).vd = NULL;
    }
    if (FM_PO>1) std::cout << "Done..." << std::flush;
  }

  // for each vertex, find all neighboring vertices
  std::list<int> nbrs;
  std::list<int>::iterator iter;
  for (int i=0;i<nv;i++) {
    nbrs.clear();
    for (int j=0;j<v(i).nf;j++) {
      sla::Vec3i ind = v(i).f(j).getVertexIndices(vd);
      for (int k=0;k<3;k++) if (ind(k) != i) nbrs.push_back(ind(k));
    }
    if (nbrs.size() > 0) {
      nbrs.sort();
      nbrs.unique();
      v(i).nv = nbrs.size();
      v(i).vd = new FlexVertex<T>*[v(i).nv];
      iter = nbrs.begin();
      for (int j=0;j<v(i).nv;j++,iter++) v(i).vd[j] = &(v(*iter));
    }
  }

  haveVertexToVertex = true;

  if (FM_PO>1) std::cout << "Done." << std::endl << std::flush;  
}

// Find face-to-face mappings
template <class T> 
inline void FlexMesh<T>::findFaceToFace() {
  if (!haveVertexToFace) findVertexToFace();

  if (FM_PO>1) std::cout << "Finding face-to-face mappings..." 
			 << std::flush;

  // clean up previously-allocated memory/mappings
  if (haveFaceToFace){
    if (FM_PO>1) std::cout << "deleting old mappings..." << std::flush;
    for (int i=0;i<nf;i++) f(i).nf = 0;
    if (FM_PO>1) std::cout << "Done..." << std::flush;
  }
  
  // clear face flags
  for (int i=0;i<nf;i++) f(i).flag = -1;

  // scan through faces
  for (int i=0;i<nf;i++) {
    sla::Vec3i ind0 = getFace(i);
    for (int j=0;j<3;j++) {
      for (int k=0;k<f(i).v(j).nf;k++) {
	sla::Vec3i ind1 = f(i).v(j).f(k).getVertexIndices(vd);
	int count = 0;
	for (int i0=0;i0<3;i0++) for (int i1=0;i1<3;i1++) {
	  if (ind0(i0) == ind1(i1)) count++;
	}
	if (count == 2 && f(i).v(j).f(k).flag != i) {
	  if (f(i).nf > 2) {
	    std::cerr << "Error, too many faces!\n" << std::flush;
	    std::cerr << "Face " << i << ":\n" << getFace(i) << "\n"
		      << "Face: " << f(i).v(j).f(k).getVertexIndices(vd) 
		      << "\n";
	    exit(1);
	  }
	  f(i).fd[f(i).nf] = &(f(i).v(j).f(k));
	  f(i).nf++;
	}
	f(i).v(j).f(k).flag = i;
      }
    }
  }
  
  haveFaceToFace = true;
  
  if (FM_PO>1) std::cout << "Done." << std::endl << std::flush;  
}

// Find vertex-to-edge mappings while defining edges themselves
template <class T> 
inline void FlexMesh<T>::findEdges() {
  if (!haveVertexToVertex) findVertexToVertex();
  if (!haveVertexToFace) findVertexToFace();

  if (FM_PO>1) std::cout << "Finding edges..." 
			 << std::flush;

  // reset links if needed
  if (haveEdges) {
    if (FM_PO>1) std::cout << "deleting old mappings..." << std::flush;
    if (ne > 0) delete [] ed;
    for (int i=0;i<nv;i++) if (v(i).ne > 0) { delete [] v(i).ed; v(i).ne = 0; }
    if (FM_PO>1) std::cout << "Done..." << std::flush;
  }

  // compute number of edges in mesh and allocate memory for them
  nn = 0; for (int i=0;i<nf;i++) nn += f(i).nf;
  ne = 3*nf-nn/2;
  if (FM_PO>1) std::cout << "expect " << ne << "..." << std::flush;
  ed = new FlexEdge<T>[ne];

  // Go through and allocate memory for vertex-edge links
  for (int i=0;i<nv;i++) if (v(i).nv > 0) {
    v(i).ne = v(i).nv;
    v(i).ed = new FlexEdge<T>*[v(i).ne];
    for (int j=0;j<v(i).ne;j++) v(i).ed[j] = NULL;
  }
  
  // Current working edge
  int ce = 0;

  // Scan over vertices i
  for (int i=0;i<nv;i++) {
    // Scan over neighboring vertices j1 not already touched
    for (int j1=0;j1<v(i).nv;j1++) if (v(i).ed[j1] == NULL) {
      // Find corresponding index j2 for vertex i in neighboring vertex j1
      int j2 = -1;
      for (int k=0;k<v(i).v(j1).nv;k++) {
	if (&(v(i)) == &(v(i).v(j1).v(k))) { j2 = k; break; }
      }
      // Exit on failure
      if (j2 == -1) {
	std::cerr << "Bad vertex pair...Exiting.\n" << std::flush;
	exit(1);
      }
      if (v(i).v(j1).ed[j2] != NULL) {
	std::cerr << "Bad vertex pair...Exiting.\n" << std::flush;
	exit(1);
      }
      // Add this edge as a new edge
      v(i).ed[j1] = v(i).v(j1).ed[j2] = &(e(ce));
      e(ce).vd[0] = &(v(i)); e(ce).vd[1] = &(v(i).v(j1));
      e(ce).x = T(0.5)*(v(i).x + v(i).v(j1).x);
      ce++;
    }
  }
  ne = ce; // just in case, reset to actual number of edges

  // Find neighboring triangles to each edge
  for (int i=0;i<ne;i++) {
    for (int j=0;j<e(i).v(0).nf;j++) {
      for (int k=0;k<3;k++) {
	if (&(e(i).v(0).f(j).v(k)) == &(e(i).v(1))) {
	  e(i).fd[e(i).nf] = &(e(i).v(0).f(j));
	  e(i).nf++;
	}
      }
    }
  }

  haveEdges = true;

  if (FM_PO>1) std::cout << "found " << ce << "...Done." 
			 << std::endl << std::flush;
}

// Store current vertex positions as reference positions
template <class T> 
inline void FlexMesh<T>::storeReferenceData() {
  for (int i=0;i<nv;i++) { v(i).x0 = v(i).x; v(i).n0 = v(i).n; }
  for (int i=0;i<nf;i++) f(i).n0 = f(i).n;
}

// Compute the color-gradient-based edge weights
template <class T> 
inline void FlexMesh<T>::findColorWeights() {
  T coeff = T(0.01);
  if (!haveEdges) findEdges();
  for (int i=0;i<ne;i++) {
    e(i).w = exp(-coeff*T((e(i).v(1).c - e(i).v(0).c).normSqr()));
  }
}

// Find the gradient of the potential function with respect to the vertex
// positions
template <class T> 
inline void FlexMesh<T>::findGradient() {
  T coeff = T(0.1); // confidence given to measurement potential
  for (int i=0;i<nv;i++) {
    v(i).dpdx = coeff*(v(i).x - v(i).x0);
    if (!v(i).boundary) for (int j=0;j<v(i).nv;j++)
      v(i).dpdx += v(i).e(j).w*(v(i).x - v(i).v(j).x);
  }
}

// Take hill climbing step
template <class T> 
inline void FlexMesh<T>::takeHillClimbingStep() {
  T stepSize = T(0.05);
  for (int i=0;i<nv;i++) {
    v(i).x -= stepSize*v(i).dpdx;
  }
}


#endif
