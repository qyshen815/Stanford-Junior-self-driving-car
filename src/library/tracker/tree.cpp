/*
 * libss - efficient posterior approximation using Scaling Series approach 
 * Copyright (C) 2004-2006, Anna Petrovskaya (libss@eonite.com)
 * All rights reserved.
 *
 */

#include "tree.h"

void
Tree1D::add(double d,int id) 
{
  m.insert(TreeMMDI_Pair(d,id));
}

TreeIter
Tree1D::begin() {
  return m.begin();
}

TreeIter
Tree1D::end() {
  return m.end();
}

TreeIter
Tree1D::lowerBound(double l) {
  return m.lower_bound(l);
}




void
Tree1DSearch::search(Tree1D &tree,double start,double end) {
  t=&tree;
  i=t->lowerBound(start);
  u=end;
}

int 
Tree1DSearch::next() {
  if (i==t->end()) 
    return -1;
  if (i->first>u) 
    return -1;
  int r=i->second;
  i++;
  return r;
}




TreeMD::TreeMD(int dim) {
  N=dim; 
}

TreeMD::~TreeMD() {
}
  
void
TreeMD::addPoint(double *loc,int id) {
  points.push_back(TreeMD_Entry(loc,id));
  t1.add(*loc,points.size()-1);
}




TreeMDSearch::TreeMDSearch(TreeMD *tmd) {
  this->tmd=tmd;
}

TreeMDSearch::~TreeMDSearch() {
}
  
void
TreeMDSearch::startSearch(double *lowB, double *upB) {
  this->lowB=lowB;
  this->upB=upB;
  ti=tmd->t1.lowerBound(*lowB);
}

int
TreeMDSearch::next() {
  TreeIter e=tmd->t1.end();
  while (ti!=e) {
    if (ti->first>*upB) 
      return -1;
    TreeMD_Entry ent=tmd->points[ti->second];
    double *loc=ent.first;
    bool match=true;
    for (int i=1;i<tmd->N;i++) {
      if (loc[i]<lowB[i]||loc[i]>upB[i]) {
	match=false;
	break;
      }
    }
    ti++;
    if (match) {
      return ent.second;
    }
  }
  return -1;
}

