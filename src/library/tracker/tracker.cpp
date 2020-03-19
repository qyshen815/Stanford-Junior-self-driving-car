#include "tracker.h"

using namespace dgc;
using std::cout;
using std::endl;

/********************************************************************
 *
 * Dynamic Obstacle
 *
 ********************************************************************/

DynamicObstacle::DynamicObstacle(double len, double width, double height) 
{
  dim.v[0]=len/2.0;
  dim.v[1]=width/2.0;
  zDim=height/2.0;
  sinYaw=0;
  cosYaw=0;
  score=0;
  wt=0;
  dNearby=1; //0.5; //was 1
  vel=0;
  initCorners2D();
  obsID=0;
  //dynamic by default, for static DGC_STATIC_OBSTACLE
  type=DGC_DYNAMIC_OBSTACLE;
  rearWheelOffset=2;
}

void 
DynamicObstacle::toMsg(DynObsMsgP wrapperMsg) 
{
  PerceptionDynamicObstacle *msg=&(wrapperMsg->msg);
  msg->id=obsID;
  msg->x=origin.v[0];
  msg->y=origin.v[1];
  msg->direction=yaw;
  msg->width=dim.v[1]*2;
  msg->length=dim.v[0]*2;
  msg->velocity=vel;
  msg->confidence=1;
  msg->obstacleType=type;
  wrapperMsg->pts=pts;
}

void 
DynamicObstacle::fromMsg(DynObsMsgP wrapperMsg) {
  PerceptionDynamicObstacle *msg=&(wrapperMsg->msg);	
  Vec2 pos(msg->x, msg->y);
  vel=msg->velocity;
  moveTo(pos,msg->direction);
  type=msg->obstacleType;
}

void 
DynamicObstacle::moveTo(Vec2 &pos, double inYaw) 
{
  origin=pos;
  yaw=inYaw;
  sinYaw=sin(yaw);
  cosYaw=cos(yaw);
  computeWorldCorners2D();
}

void 
DynamicObstacle::drive(double dt) 
{
  //rotate around rear wheels
  Vec2 toRearWheels(-rearWheelOffset, 0);
  toRearWheels=toGlobal(toRearWheels);
  moveTo(toRearWheels, yaw+dYaw);
  
  //translate forward
  Vec2 dPos(rearWheelOffset+vel*dt,0);
  Vec2 newPos = toGlobal(dPos);
  moveTo(newPos, yaw);
  
  //rotate again
  toRearWheels.set(-rearWheelOffset, 0);
  toRearWheels=toGlobal(toRearWheels);
  moveTo(toRearWheels, yaw+dYaw2);
  dPos.set(rearWheelOffset,0);
  newPos=toGlobal(dPos);
  moveTo(newPos, yaw);
}

void 
DynamicObstacle::markOnAG(AngleGrid *ag, double pad) 
{
  Vec2 pts[4];
  pts[0].set(-dim.v[0]-pad, -dim.v[1]-pad);
  pts[1].set(dim.v[0]+pad, -dim.v[1]-pad);
  pts[2].set(-dim.v[0]-pad, dim.v[1]+pad);
  pts[3].set(dim.v[0]+pad, dim.v[1]+pad);
  for(int i=0; i<4; i++) {
    pts[i]=toGlobal(pts[i]);
  }
  Vec2i agCoords = ag->getCoordRange(pts, 4);
  for(int i=agCoords.v[0]; i<=agCoords.v[1]; i++) {
    Vec2 p = ag->getWorldCoords(i);
    if (externalDist(p) <= pad) {
      ag->get(i)->obsMarked = true;
    }
  }
}

void 
DynamicObstacle::gls_render() 
{
  Vec3 col(score, 1 - score, 0);
  gls_render(col);
}

void 
DynamicObstacle::gls_render(Vec3 &color) 
{
  Vec3 loc(origin.v[0] - lasertrack_gls->origin_x, 
	   origin.v[1] - lasertrack_gls->origin_y, 0);
  double sx, sy, sz;
  Vec3 dim3(dim.v[0], dim.v[1], zDim);
  
  glsPushMatrix(lasertrack_gls);
  glsTranslatef(lasertrack_gls, loc.v[0], loc.v[1], 0);
  glsRotatef(lasertrack_gls, dgc_r2d(yaw), 0, 0, 1);
  glsPointSize(lasertrack_gls, 5);
  glsLineWidth(lasertrack_gls, 2.0);
  glsBegin(lasertrack_gls, GLS_LINES);
  glsColor3f(lasertrack_gls, color.v[0], color.v[1], color.v[2]);
  for(sx = -1; sx <= 1; sx += 2) {
    for(sy = -1; sy <= 1; sy += 2) {
      for(sz = -1; sz <= 1; sz += 2) {
	glsVertex3f(lasertrack_gls,
			sx * dim3.v[0],
			sy * dim3.v[1],
			sz * dim3.v[2]);
      }
    }
  }
  for(sx = -1; sx <= 1;sx += 2) {
    for(sz = -1; sz <= 1;sz += 2) {
      for(sy = -1; sy <= 1;sy += 2) {
	glsVertex3f(lasertrack_gls,
			sx * dim3.v[0],
			sy * dim3.v[1],
			sz * dim3.v[2]);
      }
    }
  }
  for(sz = -1; sz <= 1; sz += 2) {
    for(sy = -1; sy <= 1; sy += 2) {
      for(sx = -1; sx <= 1; sx += 2) {
	glsVertex3f(lasertrack_gls,
			sx * dim3.v[0],
			sy * dim3.v[1],
			sz * dim3.v[2]);
      }
      }
  }
  glsVertex3f(lasertrack_gls, -dim3.v[0], -dim3.v[1], dim3.v[2]);
  glsVertex3f(lasertrack_gls, dim3.v[0], 0, dim3.v[2]);
  glsVertex3f(lasertrack_gls, -dim3.v[0], dim3.v[1], dim3.v[2]);
  glsVertex3f(lasertrack_gls, dim3.v[0], 0, dim3.v[2]);
  glsEnd(lasertrack_gls);
  glsPointSize(lasertrack_gls, 1);
  glsLineWidth(lasertrack_gls, 1.0);
  glsPopMatrix(lasertrack_gls);
}

Vec2 
DynamicObstacle::toLocal(Vec2 &p) 
{
  Vec2 tp = p;
  tp.sub(origin);
  Vec2 ret=tp;
  ret.v[0]=cosYaw*tp.v[0]+sinYaw*tp.v[1];
  ret.v[1]=-sinYaw*tp.v[0]+cosYaw*tp.v[1];
  return ret;
}

Vec2 
DynamicObstacle::toGlobal(Vec2 &p) {
  Vec2 tp = p;
  tp.v[0]=cosYaw*p.v[0]-sinYaw*p.v[1];
  tp.v[1]=sinYaw*p.v[0]+cosYaw*p.v[1];
  tp.add(origin);
  return tp;
}

bool 
DynamicObstacle::containsPoint(Vec2 &p) {
  Vec2 tp = toLocal(p);
  tp.fabs();
  tp.sub(dim);
  for(int i=0;i<2;i++) {
    if (tp.v[i]>0) return false;
  }
  return true;
}

int 
DynamicObstacle::nearbyPoint(Vec2 &p) {
  Vec2 tp = toLocal(p);
  tp.fabs();
  tp.sub(dim);
  for(int i=0;i<2;i++) {
    if (tp.v[i]>=dNearby) return DO_FAR;
  }
  for(int i=0;i<2;i++) {
    if (tp.v[i]>0) return DO_NEAR;
  }
  return DO_INSIDE;
}

void 
DynamicObstacle::setSignificantSides(Vec2 &viewPt) {
  Vec2 v = toLocal(viewPt);
  for(int i=0; i<4; i++) significantSides[i]=false;
  if (v.v[0]>dim.v[0]) significantSides[0]=true;
  if (v.v[1]>dim.v[1]) significantSides[1]=true;
  if (v.v[0]<-dim.v[0]) significantSides[2]=true;
  if (v.v[1]<-dim.v[1]) significantSides[3]=true;
}

bool 
DynamicObstacle::visibleSurfacePoint(Vec2 &worldPt) 
{ //assumed to be interior point
  Vec2 v = toLocal(worldPt);
  double surfaceDepth=0.4;
  if (significantSides[0] && v.v[0]>dim.v[0]-surfaceDepth) return true;
  if (significantSides[1] && v.v[1]>dim.v[1]-surfaceDepth) return true;
  if (significantSides[2] && v.v[0]<-dim.v[0]+surfaceDepth) return true;
  if (significantSides[3] && v.v[1]<-dim.v[1]+surfaceDepth) return true;
  return false;
}


double 
DynamicObstacle::externalDist(Vec2 &p) {
  Vec2 tp = toLocal(p);
  tp.fabs();	
  tp.sub(dim);
  for(int i = 0;i < 2; i++) {
    if(tp.v[i] < 0) tp.v[i] = 0;
  }
  if(tp.v[0] <= 0 && tp.v[1] <= 0) return 0;
  return tp.length();
}

void 
DynamicObstacle::initCorners2D() 
{
  corners2D[0].set(-dim.v[0],-dim.v[1]);
  corners2D[1].set(-dim.v[0],dim.v[1]);
  corners2D[2].set(dim.v[0],dim.v[1]);
  corners2D[3].set(dim.v[0],-dim.v[1]);
  boundingCorners2D[0].set(-dim.v[0]-dNearby,-dim.v[1]-dNearby);
  boundingCorners2D[1].set(-dim.v[0]-dNearby,dim.v[1]+dNearby);
  boundingCorners2D[2].set(dim.v[0]+dNearby,dim.v[1]+dNearby);
  boundingCorners2D[3].set(dim.v[0]+dNearby,-dim.v[1]-dNearby);
}

void 
DynamicObstacle::computeWorldCorners2D() 
{
  for(int i=0; i<4; i++) {
    worldCorners2D[i]=toGlobal(corners2D[i]);
    worldBoundingCorners2D[i]=toGlobal(boundingCorners2D[i]);
  }
}

bool 
DynamicObstacle::getCornerIntersect2D(Vec2 &p1, Vec2 &p2,
				      Vec2 &r1, Vec2 &r2) {
  return getIntersections2D(p1,p2,r1,r2,worldCorners2D);
}

bool 
DynamicObstacle::getBoundingIntersect2D(Vec2 &p1, Vec2 &p2,
					Vec2 &r1, Vec2 &r2) {
  return getIntersections2D(p1,p2,r1,r2,worldBoundingCorners2D);
}

/********************************************************************
 *
 * Car Model
 *
 ********************************************************************/

CarModel::CarModel(RndfLookup *inRl, DynamicObstacle *tCar) : ModelBase(4) {
  minCount=2;
  minPercent=0.3;
  rl=inRl;
  tmpCar = tCar;
  varInsideFree=0.2; //was 0.2
  //6*4*4+2*4*4=96+32=128
  varOutsideOcc=0.1; //0.03; //was 0.1
  varSurfaceFree=0.2;  //was 0.1
  normConst = 1.0/(getGaussianVal(0,varOutsideOcc)*
		   getGaussianVal(0,varSurfaceFree));
  scoreCnt=0;
}

CarModel::~CarModel() {
}

double 
CarModel::computeLikelihood(SampleP s) 
{
  sampleToCar(s, tmpCar);
  computeScore(tmpCar);
  s->d=tmpCar->score;
  return s->d;
}

void 
CarModel::sampleToCar(SampleP s, DynamicObstacle *car) {
  car->vel=s->s[3];
  Vec2 pos(s->s[0],s->s[1]);
  car->moveTo(pos,s->s[2]);
  car->score=s->d;
  car->wt=s->w;
}

void 
CarModel::carToSample(DynamicObstacle *car, SampleP s) {
  s->s[0]=car->origin.v[0];
  s->s[1]=car->origin.v[1];
  s->s[2]=car->yaw;
  s->s[3]=car->vel;
  s->w = car->score;
  s->d = car->score;
}

DynamicObstacle *
CarModel::computeMeanCar() 
{
  SampleP mean=sir->samples.computeMean();
  sampleToCar(mean, tmpCar);
  getSG()->free(mean);
  return tmpCar;
}

void 
CarModel::computeScore(DynamicObstacle *car) 
{
  scoreCnt++;
  car->score=0;
  car->setSignificantSides(vantagePoint);
  int cntSurfaceTotal=0;
  int cntSurfaceFree=0;
  int cntSurfaceOcc=0;
  int outside=0;
  int outsideOcc=0;
  
  for(int i=0; i<4; i++) {
    if (!rl->closeToRoad(car->worldBoundingCorners2D[i])) return;
  }
  Vec2i outerAngles = ag->getCoordRange(car->worldBoundingCorners2D, 4);
  Vec2i innerAngles = ag->getCoordRange(car->worldCorners2D, 4);
  outside=outerAngles.v[1]-outerAngles.v[0];
  if (outside<minCount) return;
  cntSurfaceTotal=innerAngles.v[1]-innerAngles.v[0];
  if (cntSurfaceTotal<minCount) return;
  
  //find closest corner
  int closestInd=0;
  Vec2 v=vantagePoint.addReturn(-1, car->worldCorners2D[0]);
  double minDist=v.length();
  double dist;
  for(int i=1; i<4; i++) {
    v=vantagePoint.addReturn(-1, car->worldCorners2D[i]);
    dist=v.length();
    if (dist<minDist) {
      closestInd=i;
      minDist=dist;
    }
  }
  //should have: p2 p0 p1 visible corners from left to right
  Vec2 p0=car->worldCorners2D[closestInd];
  Vec2 p1=car->worldCorners2D[(closestInd+3)%4];
  Vec2 p2=car->worldCorners2D[(closestInd+1)%4];
  int ind0=ag->worldToGrid(p0);
  int ind1=ag->worldToGrid(p1);
  int ind2=ag->worldToGrid(p2);
  if (ind1>ind0) ind1=ind0;
  if (ind2<ind0) ind2=ind0;
  
  //outside visible space
  for(int i=outerAngles.v[0]; i<innerAngles.v[0]; i++) {
    if (!ag->valid(i)) 
      continue;
    Vec2 agPt=ag->getWorldCoords(i);
    if (car->nearbyPoint(agPt)!=DO_FAR) 
      outsideOcc++;
  }
  for(int i=innerAngles.v[1]+1; i<=outerAngles.v[1]; i++) {
    if (!ag->valid(i)) 
      continue;
    Vec2 agPt=ag->getWorldCoords(i);
    if (car->nearbyPoint(agPt)!=DO_FAR) 
      outsideOcc++;
  }
  
  //rays directed at the car itself
  for(int i=innerAngles.v[0]; i<=innerAngles.v[1]; i++) {
    if (!ag->valid(i)) continue;
    Vec2 intPt;
    Vec2 agPt=ag->getWorldCoords(i);
    bool intersect=false;
    if (i<ind0) {
      intersect=segmentIntersect2D(p1,p0,vantagePoint, agPt, intPt);
    }
    if (i>ind0) {
      intersect=segmentIntersect2D(p2,p0,vantagePoint, agPt, intPt);
    }
    if (i==ind0) {
      intersect=(ag->compareRange(p0)<=0);
    }
    if (intersect) {
      if (car->nearbyPoint(agPt)==DO_INSIDE && 
	  car->visibleSurfacePoint(agPt)) cntSurfaceOcc++;
      else cntSurfaceFree++;
    } else {
      if (car->nearbyPoint(agPt)!=DO_FAR) outsideOcc++;
    }
  }	
  if (cntSurfaceOcc<minCount) return;
  double outsideOccPercent=(double)outsideOcc/(double)outside;
  double surfaceFreePercent=(double)cntSurfaceFree/(double)cntSurfaceTotal;
  double surfaceOccPercent=(double)cntSurfaceOcc/(double)cntSurfaceTotal;
  if (surfaceOccPercent<minPercent) return;
  double s=0;
  s = getGaussianVal(outsideOccPercent,varOutsideOcc)
    *getGaussianVal(surfaceFreePercent,varSurfaceFree);
  s*= normConst;
  car->score=s;
}

/********************************************************************
 *
 * DynamicObstaclePF
 *
 ********************************************************************/

DynamicObstaclePF::DynamicObstaclePF(RndfLookup *inRl, double curT, 
				     CarModel *cm) {
  init(inRl, curT, cm);
  aCopy=false;
}

DynamicObstaclePF::DynamicObstaclePF(DynamicObstaclePF *pf, 
				     CarModel *inCarModel) 
{
  init(pf->rl, pf->curTs, inCarModel);
  life = pf->life;
  badTurns = pf->badTurns;
#ifdef VERBOSE
  cout<<"badTurns: "<<pf->badTurns<<endl;
#endif
  lastGoodUpdate = pf->lastGoodUpdate;
  firstSeen = pf->firstSeen;
  lastTs = pf->lastTs;
  curTs = pf->curTs;
  dt = pf->dt;
  numNewCars = pf->numNewCars;
  initialVel = pf->initialVel;
  topScore = pf->topScore;
  N = pf->N;
  maxN = pf->maxN;
  sir->samples.clear();
  for(int i=0; i<pf->sir->samples.size(); i++) {
    sir->samples.addCopy(pf->sir->samples.get(i));
  }
  for(int i=0; i<maxN; i++) {
    *(cars[i]) = *(pf->cars[i]);
    double deltaT = doubleRand(-0.1, 0.1);
    cars[i]->drive(deltaT);
  }
  *mean = *(pf->mean);
  *est = *(pf->est);
  aCopy=true;
}

void 
DynamicObstaclePF::init(RndfLookup *inRl, double curT, CarModel *cm) {
  rl=inRl;
  maxN=200; 
  N=0;
  cars = new DynamicObstacle *[maxN];
  for(int i=0;i<maxN;i++) cars[i]=new DynamicObstacle(5,2,2);
  mean=new DynamicObstacle(5,2,2);
  est=new DynamicObstacle(5,2,2);
  carModel=cm;
  sir = new SIR(carModel,maxN);
  life=0;
  badTurns=0;
  numNewCars=0;
  lastTs=curT;
  curTs=curT;
  dt=0;
  firstSeen=curT;
  lastGoodUpdate=curT;
  initialVel=true;
  static int tmpID=0;
  tmpID++;
  //TODO: fix id wrapping
  obsID=tmpID%10000;
  topScore=0;
}

DynamicObstaclePF::~DynamicObstaclePF() {
  for(int i=0;i<maxN;i++) 
    delete cars[i];
  delete cars;
  delete mean;
  delete est;
  delete sir;
}

bool 
DynamicObstaclePF::full() {
  return N>=maxN;
}

void 
DynamicObstaclePF::measurementUpdate() {
  for(int i=0;i<N;i++) {
    carModel->carToSample(cars[i],
			  sir->samples.get(i));
  }
  sir->computeWeights();
  for(int i=0;i<N;i++) {
    carModel->sampleToCar(sir->samples.get(i),
			  cars[i]);
  }
}

void 
DynamicObstaclePF::motionUpdate() {
  sir->resample();
  N=maxN;
  for(int i=0;i<N;i++) {
    carModel->sampleToCar(sir->samples.get(i), cars[i]);
  }
  for(int i=0;i<N;i++) {
    if (initialVel) {
      cars[i]->vel=doubleRand(0,15);  //campus
      // cars[i]->vel=doubleRand(15,35); //freeway
    } else {
      cars[i]->vel+=dt*doubleRand(-50,50); //doubleRand(-10,10);
    }
    if (cars[i]->vel<-1) cars[i]->vel=-1;
    cars[i]->dYaw=dt*doubleRand(-0.5,0.5);
    cars[i]->dYaw2=cars[i]->dYaw; //dt*doubleRand(-0.5,0.5);
    // cars[i]->orientation.v[0]+=dt*doubleRand(-1,1);
    cars[i]->drive(dt);
  }
  initialVel=false;
}

void 
DynamicObstaclePF::update(double ts) {
  lastTs=curTs;
  curTs=ts;
  dt=curTs-lastTs;
  
  motionUpdate();
  measurementUpdate();
  computeMean();
  life++;
  topScore=maxScore();
  if (sir->zeroNormalizer || topScore<0.01) {
    badTurns++;
  } else {
    badTurns=0;
    lastGoodUpdate=ts;
  }
}

void DynamicObstaclePF::computeMean() {
  SampleP meanS=sir->samples.computeMean();
  carModel->sampleToCar(meanS, mean);
  carModel->getSG()->free(meanS);
  mean->obsID=obsID;
  
  //fix up yaw
  double meanSin=0;
  double meanCos=0;
  for(int i=0; i<N; i++) {
    meanSin+=cars[i]->sinYaw;
    meanCos+=cars[i]->cosYaw;
  }
  meanSin/=N;
  meanCos/=N;
  double meanYaw=atan2(meanSin, meanCos);
  Vec2 meanLoc=mean->origin;
  mean->moveTo(meanLoc, meanYaw);
  mean->pts.clear();
}

void 
DynamicObstaclePF::computeCenter() {
  if (numNewCars==0) 
    return;
  numNewCars=0;
  Vec2 loc;
  double yaw=0;
  for(int i=0; i<N; i++) {
    loc.add(cars[i]->origin);
    yaw+=cars[i]->yaw;
  }
  loc.scale(1.0/N);
  yaw=yaw/N;
  mean->moveTo(loc,yaw);
}

void 
DynamicObstaclePF::addNewCar(DynamicObstacle &newCar) {
  if (full()) {
    //cout<<"ERROR: pf is full"<<endl;	
    return;
  }
  (*cars[N])=newCar;
  N++;
  numNewCars++;
}

void 
DynamicObstaclePF::computeNewCars() {
  for(int cnt=0;cnt<N;cnt++) {
    carModel->computeScore(cars[cnt]);
  }
}

void DynamicObstaclePF::initialize() {
  initialVel=true;
  sir->samples.clear();
  sir->nResamples=maxN;
  for(int i=0; i<N; i++) {
    sir->samples.add(carModel->getSG()->gen());
  }
  for(int i=0;i<N;i++) {
    carModel->carToSample(cars[i],
			  sir->samples.get(i));
  }
  sir->weightsNormalize();
  if (sir->zeroNormalizer) {
    cout<<"ERROR: zero normalizer during init"<<endl;
  }
}

double
DynamicObstaclePF::maxScore() {
  double ret=0;
  for(int i=0;i<N;i++) {
    if (cars[i]->score>ret) ret = cars[i]->score;
  }
  return ret;
}

void
DynamicObstaclePF::gls_render() {
  for(int i = 0; i < N; i++) {
    if(cars[i]->score >= 0.5) cars[i]->gls_render();
  }
}

void 
DynamicObstaclePF::gls_render_all() {
  Vec3 col(0,0,1);
  for(int i = 0; i < N; i++) cars[i]->gls_render(col);
}

bool 
DynamicObstaclePF::nearbyPF(DynamicObstaclePF *pf) {
  Vec2 dist=mean->origin;
  dist.sub(pf->mean->origin);
  if (dist.length()<2) return true;
  //if (mean->externalDist(pf->mean->origin)<2) return true;
  return false;
}

void
DynamicObstaclePF::merge(DynamicObstaclePF *pf) {
  //cout<<"BEGIN MERGE PF"<<endl;
  int pfSize=pf->N;
  if (pf->sir->samples.size() < pfSize) {
    cout<<"ERROR: pfSize: "<<pfSize<<" samples: "<<pf-sir->samples.size()<<endl;
    return;
  }
  
  for(int i=0; i<pfSize; i++) {
    sir->samples.add(carModel->getSG()->gen(pf->sir->samples.get(i)));
  }

  pfSize=sir->samples.size();

  for(int i=0; i<pfSize; i++) {
    SampleP s = sir->samples.get(i);
    s->w=s->d;
  }
  sir->weightsNormalize();
  sir->nResamples=maxN;
  N=maxN;
  sir->resample();
  sir->uniformWeights();
  for(int i=0;i<N;i++) {
    carModel->sampleToCar(sir->samples.get(i), cars[i]);
  }
  computeMean();
  //update badTurns and life?
  //cout<<"merged pf"<<endl;
}


/********************************************************************
 *
 * DOPFList
 *
 ********************************************************************/

DOPFList::DOPFList() {
};

DOPFList::~DOPFList() {
  clear();
}

void
DOPFList::init(CarModel *inCarModel) {
  carModel = inCarModel;
}

int 
DOPFList::size() {
  return list.size();
}

void
DOPFList::steal(int i) {
  DOPFVec::iterator it=list.begin();
  list.erase(it+i);
}

void
DOPFList::nuke(int i) {
  delete list[i];
  steal(i);
}

void
DOPFList::clear() {
  for(int i=0; i<size(); i++) 
    delete list[i];
  list.clear();
}

void
DOPFList::add(DynamicObstaclePF *p) { 
  list.push_back(p); 
}

void
DOPFList::addCopy(DynamicObstaclePF *p) { 
  DynamicObstaclePF *newPF = new DynamicObstaclePF(p, carModel);
  add(newPF);
}

DynamicObstaclePF *
DOPFList::get(int i) { 
  return list[i];
}

DynamicObstaclePF *
DOPFList::pop() {
  if (size()<=0) {
    cout<<"ERROR: empty PF list"<<endl;
    return NULL;
  }
  DynamicObstaclePF *ret=get(size()-1);
  list.pop_back();
  return ret;
}

bool 
DOPFList::nearbyPF(DynamicObstaclePF *pf) {
  for(int i=0; i<size(); i++) {
    if (get(i)->nearbyPF(pf)) return true;
  }
  return false;
}

bool 
DOPFList::mergeToPriorPf(int num) {
  if (num>=size()) 
    return false;
  DynamicObstaclePF *pf=get(num);
  for(int i=0; i<num; i++) {
    if (get(i)->nearbyPF(pf)) {
      get(i)->merge(pf);
      nuke(num);
      return true;
    }
  }
  return false;
}

/********************************************************************
 *
 * Tracker
 *
 ********************************************************************/

Tracker::Tracker(RndfLookup *inRl) {
  rl=inRl;
  double carX=5;
  double carY=2;
  double carZ=2;
  modelCar = new DynamicObstacle(carX,carY,carZ);
  junior = new DynamicObstacle(carX,carY,carZ);
  tmpCar = new DynamicObstacle(carX,carY,carZ);
  ts=0;
  lastTs=0;
  lastVel=0;
  doTracking=false;
  dt=0;
  carModel = new CarModel(rl, modelCar);
  maturePF.init(carModel);
  intermediatePF.init(carModel);
  babyPF.init(carModel);
  sir = new SIR(carModel,1);
  carModel->sir = sir;
  trackerInitialized=false;
  doGlobalSearch=true;
  dynObsNum=0;
  debug=false;
  newStaticObsStart=0;
}

Tracker::~Tracker() {
  delete carModel;
  delete modelCar;
  delete junior;
  delete tmpCar;
  delete sir;
}

void 
Tracker::set_robot_pose(Vec3 &pos, Vec3 &ori, 
	       double curTs, Vec2 &inLO) {
  Vec2 vel=robotPos;
  double dYaw=robotYaw;
  robotPos.set(pos.v[0], pos.v[1]);
  robotYaw=ori.v[0];
  locOffset=inLO;
  
  dt=curTs-ts;
  lastTs=ts;
  ts=curTs;
  vel.sub(robotPos);
  dYaw-=robotYaw;
  dYaw=dYaw/dt;
  double curVel=vel.length()/dt;
  double curAcc=(curVel-lastVel)/dt;
  lastVel=curVel;
#ifdef VERBOSE
  printf("dt: %fs velocity: %fm/s acc: %f dYaw: %f\n", dt, curVel, curAcc, dYaw);
  printf("x: %f y: %f yaw: %f", robotPos.v[0], robotPos.v[1], robotYaw);
  cout<<" locOffset: "<<locOffset<<endl;
#endif
  curAcc = 0;
  //set vantage point for observing obstacles
  carModel->vantagePoint.set(vantagePoint.v[0], vantagePoint.v[1]);
  carModel->ag=ag;
  carModel->agPrev=agPrev;
#ifdef VERBOSE
  cout<<"previous turn score called: "<<carModel->scoreCnt<<" times"<<endl;
#endif
  carModel->scoreCnt=0;
  
  //move junior
  junior->moveTo(robotPos, robotYaw);
  Vec2 rel(1.5,0);
  Vec2 carLoc=junior->toGlobal(rel);
  junior->moveTo(carLoc,robotYaw);
}

void
Tracker::update(double inAvgTs) {
  avgTs=inAvgTs;
  updateTracker();
  generateStaticObstacles();
}

DynObsMsgList *
Tracker::getDynObsList() { 
  return &dynObsList; 
}

void 
Tracker::generateDynObsList() {	
  dynObsNum=0;
  dynObsList.clear();
  
  //generate dynamic obs messages
  for(int i=0; i<maturePF.size(); i++) {
    DynObsMsg msg;
    maturePF.get(i)->mean->toMsg(&msg);
    dynObsList.push_back(msg);
    dynObsNum++;
  }
  
  //generate static obs messages
  for(int i=0; i<(signed)staticObsList.size(); i++) {
    DynObsMsg msg;
    staticObsList[i]->toMsg(&msg);
    dynObsList.push_back(msg);
    dynObsNum++;
  }
  
}

void
Tracker::markDynamicObsOnAG() {
  for(int i=0; i<maturePF.size(); i++) {
    maturePF.get(i)->mean->markOnAG(ag, 1); //was 0.5
  }
}		

void 
Tracker::markObsOnAG() {
  double road_angle;
  
  //mark dynamic obs on grid
  markDynamicObsOnAG();
  
  DynamicObstacle tmpObs(1, 1, 1);
  double pointSize = 0.70;
  for(int agInd = 0; agInd < ag->N; agInd++) {
    //find an unmarked obs point
    AngleCell *agCell = ag->get(agInd);
    if(agCell->obsMarked)
      continue;
    if(agCell->rng < 0.1)
      continue;
    Vec2 wPt = ag->getWorldCoords(agInd);
    if(!rl->closeToRoad(wPt, &road_angle)) 
      continue;
    
    //grow cell list
    std::vector <Vec2> cellList;
    cellList.push_back(wPt);
      int lastObsInd = agInd;
      Vec2 lastObsPt = wPt;
      AngleCell *lastObs = ag->get(lastObsInd);
      for(int obsInd = agInd + 1; obsInd < ag->N; obsInd++) {
	AngleCell *obsCell = ag->get(obsInd);
	Vec2 p = ag->getWorldCoords(obsInd);
	
	//stop if any further point will be too far from last obs point
	Vec2 p2 = ag->getWorldCoordsAtRange(obsInd, lastObs->rng);
	p2.sub(lastObsPt);
	if (p2.length()>pointSize) 
	  break;
	
	//stop if this obstacle has grown too big (from starting point)
	Vec2 p3 = ag->getWorldCoordsAtRange(obsInd, agCell->rng);
	p3.sub(wPt);
	if (p3.length()>5)
	  break;
	
	//skip if this point is already part of another obstacle
	if (obsCell->obsMarked) 
	  continue;
	
	//skip if this point is too far from previous obs point
	Vec2 v = p;
	v.sub(lastObsPt);
	if (v.length()>pointSize)
	  continue;
	
	//stop if this point is too far from starting point
	Vec2 v2 = p;
	v2.sub(wPt);
	if (v2.length()>5) 
	  break;
	
	//add this point to this obstacle
	obsCell->obsMarked = true;
	lastObsInd = obsInd;
	lastObsPt = p;
	lastObs = ag->get(lastObsInd);
	cellList.push_back(p);
      }
      
      tmpObs.moveTo(wPt, road_angle);
      Vec2 minXY, maxXY;
      for(int i = 0; i < (signed)cellList.size(); i++) {
	Vec2 loc = tmpObs.toLocal(cellList[i]);
	for(int j = 0; j < 2; j++) {
	  if(loc.v[j] < minXY.v[j]) minXY.v[j] = loc.v[j];	
	  if(loc.v[j] > maxXY.v[j]) maxXY.v[j] = loc.v[j];	
	}
      }
      double len = maxXY.v[0] - minXY.v[0] + 0.25;
      double width = maxXY.v[1] - minXY.v[1] + 0.25;
      Vec2 ctrOffset(maxXY.v[0]+minXY.v[0], maxXY.v[1]+minXY.v[1]);
      ctrOffset.scale(0.5);
      ctrOffset = tmpObs.toGlobal(ctrOffset);
      DynamicObstacle *sObs = new DynamicObstacle(len, width, 2);
      sObs->moveTo(ctrOffset, road_angle);
      
      //mark on grid
      sObs->markOnAG(ag, 0);
      sObs->ts = avgTs;
      sObs->type=DGC_STATIC_OBSTACLE;
      
      //add to list of obs
      staticObsList.push_back(sObs);
  }
#ifdef VERBOSE
  cout<<"found "<<staticObsList.size()<<" static obstacles"<<endl;
#endif
}	

void
Tracker::generateStaticObstacles() {

#ifdef VERBOSE
  Timer tm("static-obstacles");
#endif
  //clear old obstacles
  /*	for(int i=(signed)staticObsList.size()-1; i>=0; i--) {
    if (staticObsList[i]->ts > ts - 0.2) continue;
    delete staticObsList[i];
    ObstacleList::iterator it=staticObsList.begin();
    staticObsList.erase(it+i);
    }
  */
  for(int i=0; i<(signed)staticObsList.size(); i++) {
    delete staticObsList[i];
  }
  staticObsList.clear();
  
  //generate new obstacles
  newStaticObsStart = staticObsList.size();
  markObsOnAG();
  }

void 
Tracker::addPointToObs(Vec2 &pt, DynamicObstacle *obs) {
  Vec2 ori2D(obs->origin.v[0], obs->origin.v[1]); 
  Vec2 relativeToOrigin = pt;
  relativeToOrigin.sub(ori2D);
  obs->pts.push_back(relativeToOrigin);
}

void
Tracker::addObstaclePoint(Vec2 &pt) {
  bool addedToObs = false;
  for(int i = 0; i < maturePF.size(); i++) {
    DynamicObstacle *obs = maturePF.get(i)->mean;
    if(obs->nearbyPoint(pt) != DO_FAR) {
      addPointToObs(pt, obs);
      addedToObs = true;
      break;
    }
  }
  if(addedToObs) 
    return;
  for(int i = newStaticObsStart; i < (signed)staticObsList.size(); i++) {
    if(staticObsList[i]->externalDist(pt) < 0.25) {
      addPointToObs(pt, staticObsList[i]);
      break;
    }
  }
}

void 
Tracker::globalSearchInner(AngleGrid *agPtr) {
  double roadAngle;
  
  for(int a=0; a < agPtr->N; a++) {
    if(!agPtr->get(a)->markedAsCleared) 
      continue;
    Vec2 wPt = agPtr->getWorldCoords(a);
    
    if(!rl->closeToRoad(wPt, &roadAngle)) 
      continue;
    if(nearExistingPF(wPt)) 
      continue;
    DynamicObstaclePF *pf = getNewPF(wPt);
    
    Vec2 delta;
    tmpCar->moveTo(wPt,0);
    for(int i=0; i < 10; i++) {
      delta.v[0] = doubleRand(-tmpCar->dim.v[0], tmpCar->dim.v[0]);
      delta.v[1] = doubleRand(-tmpCar->dim.v[1], tmpCar->dim.v[1]);
      double sYaw = sampleYaw(roadAngle);
      Vec2 carLoc = tmpCar->toGlobal(delta);
      tmpCar->moveTo(carLoc,sYaw);
      pf->addNewCar(*tmpCar);
    }
  }
}

double
Tracker::sampleYaw(double roadYaw) {
  double dAng=doubleRand(-0.2,0.2);
  return roadYaw+dAng;
}

void
Tracker::globalSearch() {
#ifdef VERBOSE
  Timer sTimer("global-search");
#endif
  globalSearchInner(ag);
  globalSearchInner(agPrev);
  for(int i=0;i<babyPF.size();) {
    //babyPF.get(i)->gls_render_all();
    babyPF.get(i)->computeNewCars();
    //babyPF.get(i)->gls_render_all();
    if (babyPF.get(i)->maxScore() <=0.5) {
      babyPF.nuke(i);
      continue;
    }
    i++;
  }
  for(int i=0;i<babyPF.size();i++) {
    babyPF.get(i)->initialize();
  }
}	

DynamicObstaclePF *
Tracker::getNewPF(Vec2 &loc) {
  int sz=babyPF.size();
  for(int i=0; i<sz; i++) {
    babyPF.get(i)->computeCenter();
  }
  Vec2 v;
  for(int i=0; i<babyPF.size(); i++) {
    v=loc;
    v.sub(babyPF.get(i)->mean->origin);
    if (v.length()<3) return babyPF.get(i);
  }
  DynamicObstaclePF *pf = new DynamicObstaclePF(rl,ts, carModel);
  babyPF.add(pf);
  return pf;
}

bool 
Tracker::nearExistingPF(Vec2 &loc) {
  for(int i=0; i<maturePF.size(); i++) {
    DynamicObstacle *pfCar;
    pfCar=maturePF.get(i)->mean;
    if (pfCar->externalDist(loc)<2) return true;
  }
  for(int i=0; i<intermediatePF.size(); i++) {
    DynamicObstacle *pfCar;
    pfCar=intermediatePF.get(i)->mean;
    if (pfCar->externalDist(loc)<2) return true;
  }
  return false;
}

bool
Tracker::observedByThisSensor(DynamicObstaclePF *pf) {
  DynamicObstacle mean(5,2,2);
  mean = *(pf->mean);
  mean.drive(ts - pf->lastTs);
  int cnt=0;
  for(int i = 0; i < ag->N; i++) {
    Vec2 wPt = ag->getWorldCoords(i);
    if (mean.nearbyPoint(wPt) != DO_FAR) cnt++;
  }
  if (cnt>3)
    return true;
  return false;
}

void 
Tracker::updateTracker() {
  if (!doTracking) {
    maturePF.clear();
    intermediatePF.clear();
  } else {
    //update mature & intermediate pf
#ifdef VERBOSE
    Timer pfUpdateTm("update-pfs");
#endif	
    for(int i=0; i<maturePF.size(); i++) {
      DynamicObstaclePF *pf = maturePF.get(i);
      //if (!observedByThisSensor(pf)) continue;
      pf->update(ts);
      pf->mean->ts=avgTs;
    }
    for(int i=0; i<intermediatePF.size(); i++) {
      DynamicObstaclePF *pf = intermediatePF.get(i);
      //if (!observedByThisSensor(pf)) continue;
      pf->update(ts);
      pf->mean->ts=avgTs;
    }
    
      //clean pf's
    for(int i=0; i<maturePF.size();) {
      DynamicObstaclePF *pf = maturePF.get(i);
#ifdef VERBOSE
      cout<<"mature: "<<pf->topScore<<" badTurns: "<<pf->badTurns;
      cout<<" pos: "<<pf->mean->origin<<" vel: "<<pf->mean->vel;
      cout<<"TS: "<<ts<<" lgu: "<<pf->lastGoodUpdate<<endl;
      //			printf(" TS: %f\n", ts);
#endif
      if (pf->aCopy) {
	pf->aCopy=false;
	cout<<"pf copy: "<<pf->badTurns<<endl;
	if (pf->badTurns>0) {
	  maturePF.nuke(i);
	  continue;
	}
      }
      if (ts-pf->lastGoodUpdate>0.6 || pf->badTurns>5) { 
	maturePF.nuke(i);
	continue;
      } 
      if (maturePF.mergeToPriorPf(i)) {
	continue;
      }
      i++;
    }
    for(int i=0; i<intermediatePF.size();) {
      DynamicObstaclePF *pf = intermediatePF.get(i);
#ifdef VERBOSE
      cout<<"intermediate: "<<pf->topScore<<" "<<pf->mean->vel;
      printf(" TS: %f\n", ts);
#endif
      if (ts-pf->lastGoodUpdate>0.2 || pf->badTurns>0) { 
	//			if (pf->badTurns>0) {
	intermediatePF.nuke(i);
	continue;
      } 
      if (pf->mean->vel<5) {
	intermediatePF.nuke(i);
	continue;
      }
      //			if (ts-pf->firstSeen>0.3 ) {
      if (pf->life>5) {
	intermediatePF.steal(i);
	maturePF.add(pf);
	maturePF.mergeToPriorPf(maturePF.size()-1);
	continue;
      }
      if (intermediatePF.mergeToPriorPf(i)) {
	continue;
      }
      i++;
    }
  }
  
  //global search
  if (doGlobalSearch) {
    globalSearch();
#ifdef VERBOSE
    cout<<"adding "<<babyPF.size()<<" new pfs"<<endl;
#endif
    while(babyPF.size()>0) {
      intermediatePF.add(babyPF.pop());
    }
  }
#ifdef VERBOSE
  cout<<"maturePF: "<<maturePF.size();
  cout<<" interPF: "<<intermediatePF.size();
  cout<<endl;
#endif
}
  
void 
Tracker::gls_render(int displayAllCars, int displayInter) {
  Vec3 col(0, 0, 1);
  
  if (displayInter) {
    col.set(0.5, 0.5, 0.5);
    for(int i = 0; i < intermediatePF.size(); i++) {
      intermediatePF.get(i)->mean->gls_render(col);
      if(displayAllCars) intermediatePF.get(i)->gls_render();
    }
  }
  
  for(int i=0; i<maturePF.size(); i++) {
    col.set(1,0,1);
    maturePF.get(i)->mean->gls_render(col);
    col.set(0,1,0);
    if (displayAllCars) maturePF.get(i)->gls_render();
  }
  
  for(int i=0; i<(signed)staticObsList.size(); i++) {
    col.set(0,1,1);
    staticObsList[i]->gls_render(col);
  }   
}

