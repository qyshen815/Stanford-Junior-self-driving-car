/*
 * libss - efficient posterior approximation using Scaling Series approach 
 * Copyright (C) 2004-2006, Anna Petrovskaya (libss@eonite.com)
 * All rights reserved.
 *
 */
#ifndef LIBSS_H
#define LIBSS_H

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <list>
#include <vector>

#include "vectors.h"

typedef double * StateP;

typedef struct {
  double     w;
  double     d;
  // int        N;
  StateP     s;
} SampleStruct;

typedef SampleStruct * SampleP;


class SampleGen {
public:
  SampleGen(int dimentionality,int extra=0);
  
  SampleP gen();
  SampleP gen(SampleP s);
  void free(SampleP s);
  
  double dist(SampleP s1, SampleP s2);
  void moveG(SampleP s, double sd);
  void moveU(SampleP s, double md);
  
  void copy(SampleP s, SampleP d);
  
  int N;
  int T;
  
  void print(SampleP s);

};

typedef std::vector<SampleP> SamplePVec;

class SampleList {

public:
  SampleList(SampleGen *sg);
  ~SampleList();
  
  SampleP get(int i);
  int size();

  // note: this does not copy Sample, and instead keeps the pointer 
  // to Sample; the Sample structure pointed by SampleP will be deleted
  // on clear (using sg->free)
  void add(SampleP s);
  void addCopy(SampleP s);
  void printMinMax();
  void printStats(int col);
  void getMinMax(SampleP minSample, SampleP maxSample);
  SampleP computeMean();
  
  // remove all entries, and delete all Sample structures pointed to by them
  void clear();
  
  // swaps sample lists with another SampleList
  void swapSamples(SampleList *sl);
  
  void writeToFile(const char *fileName);
  
  SamplePVec *samples;
  
  SampleGen *sg;
};

class TempSamples {
public:
  TempSamples(SampleGen *sg);
  ~TempSamples();
  SampleP get();
  
  //private:
  SampleGen *sg;
  std::list<SampleP> lSamples;
};

class Diversity {
 public:
  void clearStats();
  void addParticle(double w);
  void printStats();
  
  double maxWeight;
  double validParticleCutoff;
  double weightSum;
  double highWeight;
  int totalParticles;
  int validParticles;
  
  double delta;
};

class ModelBase {
 public:
  ModelBase(int dimentionality, int extra=0);
  virtual ~ModelBase();
  
  double error;

  SampleGen *getSG();
  
  void setError(double err);
  // does whatever pre-computations to be able to perform 
  // computeLikelihood at this error
  virtual void setupError();
  
  virtual double computeLikelihood(SampleP s) = 0;
  virtual void setupDiversity(Diversity *d);
  // returns weather to increase precision or remain at current precision
  virtual bool checkDiversity(Diversity *d);
  
  virtual void printSample(SampleP s);
  virtual void printMinMax(SampleList &lst);
  
  // called by spaceUniformResample once it moved a partice
  // should return if this particle is still in valid location
  // note: this method is allowed to slightly modify the location of the particle
  virtual bool withinBounds(SampleP s);
  bool checkBounds;
  
 private:
  SampleGen *sg;
};

class SIR {
 public:
  SIR(ModelBase *m, int startSamples);
  ~SIR();
  
  void computeWeights();
  void computeUnnormalizedWeights(Diversity *d);
  void computeWeights(Diversity *d);
  void weightsNormalize();
  void uniformWeights();
  
  void resample();
  void randMove(double sd);
  SampleList *sampleDistribution(int num);
  
  int highestWeightInd();
  
  void freeSamples();
  
  ModelBase *model;
  
  SampleList samples;
  //SampleList newSamples;
  
  int nResamples;
  bool zeroNormalizer;
};

class SSPF: public SIR {
 public:
  SSPF(ModelBase *m, int startSamples);
  ~SSPF();
  
  void spaceUniformResample();
  void setupDiversity();
  
  void spaceUniformWeightNormalize();
  
  void setDeltaRange(double deltaStart, double deltaEnd);
  void setParticlesPerDelta(int ppd);
  void setDiversity(Diversity *div);
  
  void runSSIS();
  
  void setExitOnTimeLimit(clock_t inLimitTime);
  void setExitOnSampleLimit(int inLimitSamples);
  
  //private:
  double delta;
  double zoom;
  
  double deltaDesired;
  
  double uniformWeightNormalizeCutoff;
  
  int particlesPerDelta;
  Diversity *diversity;
  
  int zoomRetry;
  int zoomRetryMax;
  
  bool exitOnTimeLimit;
  clock_t limitTime;
  bool exitOnSampleLimit;
  int limitSamples;
  bool reachedExitCondition();
};


void   dblArrCopy(int N, double *src, double *dst);
void   dblArrMult(int N, double *d, double f);
void   dblArrAdd(int N, double *d, double *a, double f);
void   dblArrAdd(int N, double *d, double *a);
void   dblArrSub(int N, double *d, double *a);
void   dblArrZero(int N, double *d);
double dblArrLenSq(int N, double *d);
double dblArrLen(int N, double *d);
void   dblArrPrint(int N, double *d);

#endif
