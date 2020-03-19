/*
 * libss - efficient posterior approximation using Scaling Series approach 
 * Copyright (C) 2004-2006, Anna Petrovskaya (libss@eonite.com)
 * All rights reserved.
 *
 */
#ifndef LIBSS_TREES_H
#define LIBSS_TREES_H

#include <map>
#include <vector>

typedef std::multimap< double, int > TreeMMDI;
typedef std::pair< double, int > TreeMMDI_Pair;
typedef TreeMMDI::iterator TreeIter;

class Tree1D {
 public:

  TreeMMDI m;

  void     add(double d,int id);
  TreeIter begin();
  TreeIter end();
  TreeIter lowerBound(double l);

};


struct Tree1DSearch {

  Tree1D    *t;
  TreeIter   i;
  double     u;

  void search(Tree1D &tree, double start, double end);
  int next();

};

typedef std::pair<double*,int> TreeMD_Entry;

/*
 * this should use RTrees, once it's figured out how to make
 * them fast, in the mean time, this uses Tree1D on one of 
 * the dimentions (the first one), and liner search on the 
 * rest of the items
 */

class TreeMD {


 public:
  TreeMD(int dim);
  ~TreeMD();
  
  void addPoint(double *loc,int id);

  //private:
  int N;
  std::vector< TreeMD_Entry > points;
  Tree1D t1;
};

class TreeMDSearch {

 public:
  TreeMDSearch(TreeMD *tmd);
  ~TreeMDSearch();
  
  // searches can be restarted with different parameters
  void startSearch(double *lowB, double *upB);
      
  // returns next id in the search, or -1 if no more
  int next();

 private:
  TreeMD *tmd;
  double *lowB, *upB;
  TreeIter ti;
};

#endif
