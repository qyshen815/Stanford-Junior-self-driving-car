#include "projector.h"

using namespace std;
using namespace Eigen;

NameMapping2 Projector::generateNameMapping(const NameMapping& orig) const
{
  ROS_ASSERT(orig.size() == projectors_.size());
  NameMapping2 dmap;
  for(size_t i = 0; i < orig.size(); ++i) {
    for(size_t j = 0; j < projectors_[i].size(); ++j) {
      ostringstream oss;
      oss << orig.toName(i) << ":hashed_with_seed:" << seeds_[i][j];
      dmap.addName(oss.str());
    }
  }
  return dmap;
}

string Projector::status() const
{
  ostringstream oss;
  for(size_t i = 0; i < projectors_.size(); ++i) {
    oss << "************************************************************" << endl;
    oss << "Projectors for dspace " << i << endl;
    for(size_t j = 0; j < projectors_[i].size(); ++j) { 
      oss << projectors_[i][j].transpose() << endl;
      oss << endl;
    }
  }
  return oss.str();
}

int Projector::getNumProjections() const
{
  int num = 0;
  for(size_t i = 0; i < projectors_.size(); ++i)
    num += projectors_[i].size();
  return num;
}

uint32_t Projector::getSeed() const
{
  return seed_;
}
  

Projector::Projector(uint32_t seed, int projectors_per_dspace, const Object& obj) :
  seed_(seed)
{
  std::tr1::mt19937 seed_generator(seed);
  
  projectors_.resize(obj.descriptors_.size());
  seeds_.resize(projectors_.size());
  for(size_t i = 0; i < obj.descriptors_.size(); ++i) {
    ROS_ASSERT(obj.descriptors_[i].vector);
    
    projectors_[i].resize(projectors_per_dspace);
    seeds_[i].resize(projectors_per_dspace);
    for(size_t j = 0; j < projectors_[i].size(); ++j) {
      seeds_[i][j] = seed_generator();
      projectors_[i][j] = getProjector(seeds_[i][j], obj.descriptors_[i].vector->rows());
      //cout << "New projector: " << projectors_[i][j].transpose() << endl;
    }
  }
}

VectorXf Projector::project(const Object& obj) const
{
  VectorXf descriptors(getNumProjections());
  int idx = 0;
  for(size_t i = 0; i < obj.descriptors_.size(); ++i) {
    ROS_ASSERT(obj.descriptors_[i].vector);
    for(size_t j = 0; j < projectors_[i].size(); ++j, ++idx) {
//      cout << obj.descriptors_[i].vector->rows() << " vs " << projectors_[i][j].rows() << endl;
      descriptors(idx) = obj.descriptors_[i].vector->cast<double>().dot(projectors_[i][j]);
    }
  }
  
  return descriptors;
}

double Projector::sampleFromGaussian(std::tr1::mt19937& mersenne_twister, double stdev) {
  double sum = 0;
  for(size_t i=0; i<12; ++i) {
    sum += 2. * stdev * (double)mersenne_twister() / (double)mersenne_twister.max() - stdev;
  }
  return sum / 2.;
}
  
VectorXd Projector::getProjector(uint32_t seed, int num_dim)
{
  VectorXd projector = VectorXd::Zero(num_dim);

  // -- Each entry in the projector is drawn from the standard normal.
  // http://books.google.com/books?id=6Ewlh_lNo4sC&lpg=PP9&ots=JrJ9sqV0a5&dq=random%20projection&lr=&pg=PA2#v=twopage&q=&f=false
  std::tr1::mt19937 mersenne_twister(seed);
  for(int i = 0; i < projector.rows(); ++i)
    projector(i) = sampleFromGaussian(mersenne_twister, 1);

  projector.normalize();
  return projector;
}
