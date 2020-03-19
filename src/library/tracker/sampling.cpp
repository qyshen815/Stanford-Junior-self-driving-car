/*
 * libss - efficient posterior approximation using Scaling Series approach 
 * Copyright (C) 2004-2006, Anna Petrovskaya (libss@eonite.com)
 * All rights reserved.
 *
 */

#include <math.h>
#include "sampling.h"
#include "misc.h"
#include "tree.h"

using std::cout;
using std::endl;
using std::ofstream;

/********************************************************************
 *
 * SampleGen
 *
 ********************************************************************/

SampleGen::SampleGen(int dimentionality, int extra) {
  N=dimentionality;
  T=N+extra;
}

SampleP
SampleGen::gen() {
  SampleP s=new SampleStruct;
  s->w=0;
  s->s=new double[T];
  dblArrZero(T,s->s);
  //        cout<<"SG:ALLOC:"<<s<<endl;
  return s;
}
SampleP 
SampleGen::gen(SampleP s) {
  SampleP ns=gen();
  copy(s,ns);
  return ns;
}

void 
SampleGen::free(SampleP s) {
  //        cout<<"SG:FREE:"<<s<<endl;
  delete[] s->s;
  delete s;
}

double
SampleGen::dist(SampleP s1, SampleP s2) {
  SampleP t=gen(s1);
  dblArrSub(N,t->s,s2->s);
  double len=dblArrLen(N,t->s);
  free(t);
  return len;
}

void
SampleGen::moveG(SampleP s, double sd) {
  for (int i=0;i<N;i++) {
    s->s[i]+=gaussRand(sd);
  }
}

void
SampleGen::moveU(SampleP s, double md) {
  for (int i=0;i<N;i++) {
    s->s[i]+=doubleRand(-md,md);
  }
}

void 
SampleGen::copy(SampleP s, SampleP d) {
  d->w=s->w;
  d->d=s->d;
  dblArrCopy(T,s->s,d->s);
}

void 
SampleGen::print(SampleP s) {
  cout<<"d="<<s->d<<", w="<<s->w<<", s=";
  dblArrPrint(T,s->s);
  cout<<endl;
}

/********************************************************************
 *
 * TempSamples
 *
 ********************************************************************/

TempSamples::TempSamples(SampleGen *sg) {
  this->sg=sg;
}

TempSamples::~TempSamples() {
  while (lSamples.size() > 0) {
    SampleP s=lSamples.front();
    lSamples.pop_front();
    sg->free(s);
  }
}

SampleP
TempSamples::get() {
  SampleP s=sg->gen();
  lSamples.push_front(s);
  return s;
}

/********************************************************************
 *
 * SampleList
 *
 ********************************************************************/

SampleList::SampleList(SampleGen *sg) {
  this->sg=sg;
  samples=new SamplePVec;
}

SampleList::~SampleList() {
  clear();
  delete samples;
}

SampleP
SampleList::get(int i) {
  return (*samples)[i];
}

int
SampleList::size() {
  return samples->size();
}

void
SampleList::add(SampleP s) {
  samples->push_back(s);
}

void 
SampleList::addCopy(SampleP s) {
  SampleP newS = sg->gen(s);
  add(newS);
}

void 
SampleList::printStats(int col) {
  double l,u,mean,mean2;
  double s=get(0)->s[col];
  l=s;u=s;
  mean=mean2=0;
  for (int i=0;i<size();i++) {
    s=get(i)->s[col];
    l=std::min(l,s);u=std::max(u,s);
    mean+=s;
    mean2+=s*s;
  }
  if (size()) {
    mean/=size();
    mean2/=size();
  }
  double std1=sqrt(mean2-(mean*mean));
  double var2=0;
  for (int i=0;i<size();i++) {
    s=get(i)->s[col];
    s-=mean;
    s*=s;
    var2+=s;
  }
  if (size())
    var2/=size();
  double std2=sqrt(var2);
  cout<<"min="<<l<<" mean="<<mean<<" max="<<u<<endl;
  cout<<"std1="<<std1<<" std2="<<std2<<endl;
}

void 
SampleList::printMinMax() {
  SampleP lS=sg->gen(get(0));
  SampleP uS=sg->gen(get(0));
  for (int i=1;i<size();i++) {
    SampleP s=get(i);
    for (int j=0;j<sg->N;j++) {
      if (lS->s[j]>s->s[j]) lS->s[j]=s->s[j];
      if (uS->s[j]<s->s[j]) uS->s[j]=s->s[j];
    }
  }
  lS->w=lS->d=uS->w=uS->d=0;
  cout<<"sample list min:";
  sg->print(lS);
  cout<<"sample list max:";
  sg->print(uS);
}

void 
SampleList::getMinMax(SampleP minSample, SampleP maxSample) {
  SampleP lS=sg->gen(get(0));
  SampleP uS=sg->gen(get(0));
  for (int i=1;i<size();i++) {
    SampleP s=get(i);
    for (int j=0;j<sg->N;j++) {
      if (lS->s[j]>s->s[j]) lS->s[j]=s->s[j];
      if (uS->s[j]<s->s[j]) uS->s[j]=s->s[j];
    }
  }
  lS->w=lS->d=uS->w=uS->d=0;
  sg->copy(lS, minSample);
  sg->copy(uS, maxSample);
  sg->free(lS);
  sg->free(uS);
}

SampleP
SampleList::computeMean() {
  SampleP mean = sg->gen();
  double totalW=0;
  for (int i=0; i<size(); i++) {
    SampleP s=get(i);
    for (int j=0; j<sg->N; j++) {
      mean->s[j]+=s->w*s->s[j];
    }
    totalW+=s->w;
  }
  if (totalW==0) {
#ifdef VERBOSE
    cout<<"SampleList::computeMean: sum of weights is zero."<<endl;
#endif
  } else {
    for (int j=0; j<sg->N; j++) {
      mean->s[j]/=totalW;
    }
  }
  return mean;
}

void 
SampleList::clear() {
  while (size()>0) {
    sg->free(get(size()-1));
    samples->pop_back();
  }
}

void 
SampleList::swapSamples(SampleList *sl) {
  SamplePVec *t;
  t=sl->samples;
  sl->samples=samples;
  samples=t;
}

void 
SampleList::writeToFile(const char *fileName) {
  ofstream outFile(fileName);
  if (!outFile.is_open()) {
    cout << "Unable to open file for writing: "<<fileName<<endl;
    return;
  }
  for (int i=0;i<size();i++) {
    SampleP s=get(i);
    outFile<<"d="<<s->d<<", w="<<s->w<<", s=[ ";
    for (int j=0;j<sg->T;j++) {
      outFile<<s->s[j]<<" ";
    }
    outFile<<"]"<<endl;
  }
  outFile.close();
}

/********************************************************************
 *
 * ModelBase
 *
 ********************************************************************/

ModelBase::ModelBase(int dimentionality,int extra) {
  sg=new SampleGen(dimentionality,extra);
  error=1;
  checkBounds=false;
}

ModelBase::~ModelBase() {
        delete sg;
}

SampleGen *
ModelBase::getSG( void ) {
  return sg; 
}

void 
ModelBase::setError(double err) {
  this->error=err; 
  setupError(); 
}

void 
ModelBase::printSample(SampleP s) {
  getSG()->print(s);
}

void 
ModelBase::printMinMax(SampleList &lst) {
  SampleP minSample=getSG()->gen();
  SampleP maxSample=getSG()->gen();
  lst.getMinMax(minSample, maxSample);
  cout<<"min: ";
  printSample(minSample);
  cout<<"max: ";
  printSample(maxSample);
  getSG()->free(minSample);
  getSG()->free(maxSample);
}

bool 
ModelBase::withinBounds(__attribute__ ((unused)) SampleP s) {
  return true;
}

void 
ModelBase::setupDiversity(Diversity *d) {
  d->maxWeight=1.0;
  d->validParticleCutoff=0.5;
}

bool 
ModelBase::checkDiversity(Diversity *d) {
  d->printStats();
  return true;
}

void 
ModelBase::setupError() 
{
}

/********************************************************************
 *
 * SIR
 *
 ********************************************************************/

SIR::SIR(ModelBase *m,int startSamples):samples(m->getSG()) {
  model=m;
  nResamples=startSamples;
  for (int i=0;i<startSamples;i++) {
    samples.add(model->getSG()->gen());
  }
  uniformWeights();
  zeroNormalizer=false;
}

SIR::~SIR() {
  freeSamples();
}

void 
SIR::computeWeights() {
  int sz=samples.size();
  //#pragma omp parallel for
  for (int i=0;i<sz;i++) {
    samples.get(i)->w=model->computeLikelihood(samples.get(i));
  }
  weightsNormalize();
}

void 
SIR::computeUnnormalizedWeights(Diversity *d) {
  d->clearStats();
  int sz=samples.size();
  //#pragma omp parallel for
  for (int i=0;i<sz;i++) {
    double w=model->computeLikelihood(samples.get(i));
    samples.get(i)->w=w;
    d->addParticle(w);
  }
}

void 
SIR::computeWeights(Diversity *d) {
  computeUnnormalizedWeights(d);
  weightsNormalize();
}

void 
SIR::weightsNormalize() {
  zeroNormalizer=false;
  double t=0;
  for (int i=0;i<samples.size();i++) {
    t+=samples.get(i)->w;
  }
  if (t==0) {
    zeroNormalizer=true;
    //cout<<"SIR:weightNorm:ERROR:normalizer is 0:"<<t<<endl;
    uniformWeights();
    return;
  }
  for (int i=0;i<samples.size();i++) {
    samples.get(i)->w/=t;
  }
}

void 
SIR::uniformWeights() {
  if (samples.size()==0) {
    cout<<"WARNING: SIR::uniformWeights: no samples!"<<endl;
    return;
  }
  double w=1.0/samples.size();
  for (int i=0;i<samples.size();i++) {
    samples.get(i)->w=w;
  }
}

void 
SIR::freeSamples() {
  // don't need to do anything, samples and newSamples will clean themselves up
}

void 
SIR::resample() {
  SampleList ns(model->getSG());
  //SampleP *ns=new SampleP[nResamples];
  
  if (samples.size()==0) {
    cout<<"ERROR: SIR::resample: current sample set is empty"<<endl;
    return;
  }
  
  if (nResamples==0) {
    cout<<"WARNING: SIR::resample: generating an empty sample set"<<endl;
    samples.swapSamples(&ns);
    return;
  }
  
  double rStep=1.0/(double)nResamples;
  double r=doubleRand(0,rStep);
  int si=0;
  int nsi=0;
  double t=samples.get(si)->w;
  while (r<1.0) {
    while (t<r) {
      si++;
      if (si>samples.size()) {
	cout<<"SIR:resample:Error, ran out of samples: nSamples="<<samples.size()<<", nsi="<<nsi<<", si="<<si<<", t="<<t<<", r="<<r<<endl;
	samples.swapSamples(&ns);
	return;
      }
      t+=samples.get(si)->w;
    }
    ns.add(model->getSG()->gen(samples.get(si)));
    r+=rStep;
    nsi++;
  }
  if (nsi!=nResamples) {
    cout<<"SIR:resample:ERROR with resample: nsi="<<nsi<<", si="<<si<<", t="<<t<<", r="<<r<<endl;
  }
  
  //freeSamples();
  samples.swapSamples(&ns);
  
  //fixme: do uniformWeights()
}

SampleList *
SIR::sampleDistribution(int num) {
  SampleList *ns = new SampleList(model->getSG());
  //SampleP *ns=new SampleP[nResamples];
  
  if (samples.size()==0) {
    cout<<"ERROR: SIR::resample: current sample set is empty"<<endl;
    return ns;
  }
  
  if (num==0) {
    cout<<"WARNING: SIR::resample: generating an empty sample set"<<endl;
    return ns;
  }

  double rStep=1.0/(double)num;
  double r=doubleRand(0,rStep);
  int si=0;
  int nsi=0;
  double t=samples.get(si)->w;
  while (r<1.0) {
    while (t<r) {
      si++;
      if (si>samples.size()) {
	cout<<"SIR:resample:Error, ran out of samples: nSamples="<<samples.size()<<", nsi="<<nsi<<", si="<<si<<", t="<<t<<", r="<<r<<endl;
	return ns;
      }
      t+=samples.get(si)->w;
    }
    ns->add(model->getSG()->gen(samples.get(si)));
    r+=rStep;
    nsi++;
  }
  if (nsi!=num) {
    cout<<"SIR:resample:ERROR with resample: nsi="<<nsi<<", si="<<si<<", t="<<t<<", r="<<r<<endl;
  }
  
  //freeSamples();
  return ns;
  
  //fixme: do uniformWeights()
}

void 
SIR::randMove(double sd) {
  for (int i=0;i<samples.size();i++) {
    model->getSG()->moveG(samples.get(i), sd);
  }
}

int 
SIR::highestWeightInd() {
  double w=-1;
  int ret=-1;
  for (int i=0;i<samples.size(); i++) {
    if (w<samples.get(i)->w) {
      w = samples.get(i)->w;
      ret=i;
    }
  }
  return ret;
}

/********************************************************************
 *
 * SSPF
 *
 ********************************************************************/

SSPF::SSPF(ModelBase *m, int startSamples):SIR(m,startSamples) {
  particlesPerDelta=20;
  delta=1;
  zoom=pow(2.0,1.0/model->getSG()->N);
  diversity=NULL;
  zoomRetryMax=3;
  uniformWeightNormalizeCutoff=0.01;
  exitOnTimeLimit=false;
  exitOnSampleLimit=false;
}

SSPF::~SSPF() 
{
}

void 
SSPF::setParticlesPerDelta(int ppd) {
  particlesPerDelta=ppd;
}

void
SSPF::setDiversity(Diversity *div) {
  diversity=div; 
}

void 
SSPF::spaceUniformResample() {
  SampleGen *sg=model->getSG();
  SampleList newSamples(sg);
  int N=sg->N;
  TreeMD tmd(N);
  double *lB=new double[N];
  double *uB=new double[N];
  TreeMDSearch ts(&tmd);
  
  for (int i=0;i<samples.size();i++) {
    SampleP s=samples.get(i);
    for (int j=0;j<particlesPerDelta;j++) {
      SampleP ns=sg->gen(s);
      sg->moveU(ns,delta);
      if (model->checkBounds) {
	if (!model->withinBounds(ns)) {
	  sg->free(ns);
	  continue;
	}
      }
      for (int k=0;k<N;k++) {
	lB[k]=ns->s[k]-delta;// /2;?
	uB[k]=ns->s[k]+delta;// /2;?
      }
      ts.startSearch(lB,uB);
      if (ts.next()!=-1) {
	sg->free(ns);
	continue;
      }
      newSamples.add(ns);
    }
    tmd.addPoint(s->s,i);
  }
  
  delete lB;
  delete uB;
  
  samples.swapSamples(&newSamples);
  uniformWeights();
}

void 
SSPF::spaceUniformWeightNormalize() {
  SampleGen *sg=model->getSG();
  SampleList newSamples(sg);
  int N=sg->N;
  TreeMD tmd(N);
  double *lB=new double[N];
  double *uB=new double[N];
  TreeMDSearch ts(&tmd);
  
  bool *processed=new bool[samples.size()];
  
  double totalNewParticles=0;
  
  for (int i=0;i<samples.size();i++) {
    SampleP s=samples.get(i);
    tmd.addPoint(s->s,i);
    processed[i]=0;
  }
  for (int i=0;i<samples.size();i++) {
    SampleP s=samples.get(i);
    if (processed[i]) continue;
    
    int deltaSamplesStart=newSamples.size();
    int deltaUnprocessedParticles=0;
    int deltaTotalParticles=0;
    double deltaCurrentWeight=0;
    
    for (int k=0;k<N;k++) {
      lB[k]=s->s[k]-delta;
      uB[k]=s->s[k]+delta;
    }
    
    ts.startSearch(lB,uB);
    int nsInd;
    while ((nsInd=ts.next())!=-1) {
      deltaTotalParticles++;
      if (processed[nsInd]) continue;
      processed[nsInd]=true;
      deltaUnprocessedParticles++;
      SampleP ns=sg->gen(samples.get(nsInd));
      deltaCurrentWeight+=ns->w;
      newSamples.add(ns);
    }
    
    double deltaDensity=(double)deltaUnprocessedParticles/(double)deltaTotalParticles;
    
    if (deltaCurrentWeight<=uniformWeightNormalizeCutoff) continue;
    
    double deltaNewWeight=particlesPerDelta*deltaDensity;
    totalNewParticles+=deltaNewWeight;
    double weightMult=deltaNewWeight/deltaCurrentWeight;
    
    for (int j=deltaSamplesStart;j<newSamples.size();j++) {
      SampleP ns=newSamples.get(j);
      ns->w*=weightMult;
    }
  }
  
  delete lB;
  delete uB;
  
  samples.swapSamples(&newSamples);
  weightsNormalize();
  nResamples=(int)totalNewParticles;
}

void 
SSPF::setDeltaRange(double deltaStart, double deltaEnd) {
  delta=deltaStart;
  deltaDesired=deltaEnd;
}

void 
SSPF::setupDiversity() {
  diversity->delta=delta;
  model->setupDiversity(diversity);
}

void 
SSPF::runSSIS() {
  zoomRetry=0;
  while (delta>=deltaDesired) {
    if (!zeroNormalizer) {
      resample();
      if (true) {
	spaceUniformResample();
      } else {
	// resample leaves old weights
	uniformWeights();
	spaceUniformWeightNormalize();
	resample();
	randMove(delta/2);
      }
    }
    model->setError(delta);
    setupDiversity();
    computeWeights(diversity);
    nResamples=samples.size();
    samples.writeToFile("samples.txt");
    //resample();
    //model->getSG()->print(samples.get(highestWeightInd()));
#ifdef VERBOSE
    model->printMinMax(samples);
    cout<<"top: ";
    model->printSample(samples.get(highestWeightInd()));
#endif
    //samples.printMinMax();
    if (delta==deltaDesired) {
#ifdef VERBOSE
      model->checkDiversity(diversity);
#endif
      break;
    }
    if (model->checkDiversity(diversity)) {
      delta=delta/zoom;
      if (delta<deltaDesired) delta=deltaDesired;
      zoomRetry=0;
    } else {
      zoomRetry++;
      if (zoomRetry>zoomRetryMax) {
	cout<<"SSPF::runSSIS: cannot zoom, ran out of retrys, delta="<<delta<<", zoomRetry="<<zoomRetry<<endl;
	break;
      }
    }
    if (reachedExitCondition()) break;
  }
}

void 
SSPF::setExitOnTimeLimit(clock_t inLimitTime) {
  limitTime=inLimitTime;
  exitOnTimeLimit=true;
}

void 
SSPF::setExitOnSampleLimit(int inLimitSamples) {
  limitSamples=inLimitSamples;
  exitOnSampleLimit=true;
}

bool 
SSPF::reachedExitCondition() {
  if (zeroNormalizer) {
    cout<<"SSPF::reachedExitCondition: zeroNormalizer"<<endl;
    return true;
  }
  if (exitOnTimeLimit) {
    clock_t curTime=clock();
    if (curTime>=limitTime) {
      cout<<"SSPF::reachedExitCondition: timeLimit"<<endl;
      return true;
                }
  }
  if (exitOnSampleLimit) {
    if (samples.size()>=limitSamples) {
      cout<<"SSPF::reachedExitCondition: sampleLimit"<<endl;
      return true;
    }
  }
  return false;
}

/********************************************************************
 *
 * Diversity
 *
 ********************************************************************/

void
Diversity::clearStats() {
  totalParticles=0;
  validParticles=0;
  weightSum=0;
  highWeight=0;
}

// weights should be unnormalized (before SIR.wieghtsNormalize())
void 
Diversity::addParticle(double w) {
  totalParticles++;
  weightSum+=w;
  if (w>=validParticleCutoff) {
    validParticles++; 
  }
  if (w>highWeight) {
    highWeight=w;
  }
}

void 
Diversity::printStats() {
  cout<<"Diversity Stats: delta="<<delta<<endl;
  cout<<"maxWeight="<<maxWeight<<"\tvalidParticleCutoff="<<validParticleCutoff<<"\thighWeight="<<highWeight<<endl;
  cout<<"totalParticles="<<totalParticles<<"\tvalidParticles="<<validParticles<<"\tweightSum="<<weightSum<<endl;
}


/********************************************************************
 *
 * ????
 *
 ********************************************************************/

void
dblArrCopy(int N, double *src, double *dst) {
  for (int i=0;i<N;i++) {
    dst[i]=src[i];
  }
}

void 
dblArrMult(int N, double *d, double f) {
  for (int i=0;i<N;i++) {
    d[i]*=f;
  }
}

void
dblArrAdd(int N, double *d, double *a, double f) {
  for (int i=0;i<N;i++) {
    d[i]+=f*a[i];
  }
}

void
dblArrAdd(int N, double *d, double *a) {
  dblArrAdd(N,d,a,1);
}

void
dblArrSub(int N, double *d, double *a) {
  dblArrAdd(N,d,a,-1);
}

void
dblArrZero(int N, double *d) {
  for (int i=0;i<N;i++) {
    d[i]=0;
  }
}

double
dblArrLenSq(int N, double *d) {
  double lsq=0;
  for (int i=0;i<N;i++) {
    lsq+=d[i]*d[i];
  }
  return lsq;
}

double
dblArrLen(int N, double *d) {
  return sqrt(dblArrLenSq(N,d));
}

void
dblArrPrint(int N, double *d) {
  cout<<"[";
  for (int i=0;i<N;i++) {
    cout<<d[i]<<" ";
  }
  cout<<"]";
}

/********************************************************************
 *
 * ????
 *
 ********************************************************************/

class TestModel:public ModelBase {
public:
  TestModel():ModelBase(6) {
    sol=getSG()->gen();
    sol->s[0]=1.232;
    sol->s[1]=0.23423;
    sol->s[2]=2.4237;
    sol->s[3]=4.232;
    sol->s[4]=5.23423;
    sol->s[5]=3.4237;
  }
  SampleP sol;
  double computeLikelihood(SampleP s) {
    //Pt3 p=cPt3(st[0],st[1],st[2]);
    s->d=getSG()->dist(s,sol);
    s->w=getGaussianVal(s->d,error/2);
    return s->w;
  }
};

void 
testSSPF() {
  TestModel m;
  Diversity d;
  SSPF sspf(&m,1);
  sspf.samples.get(0)->w=1;
  sspf.uniformWeightNormalizeCutoff=0.1;
  sspf.setDeltaRange(10,0.001);
  sspf.setDiversity(&d);
  sspf.setParticlesPerDelta(6);
  sspf.runSSIS();
}

void testSIR2(SampleGen *sg) {
  TempSamples ts(sg);
}


