#ifndef PROJECTOR_H
#define PROJECTOR_H

#include <tr1/random>
#include <multibooster/multibooster.h>
#include <online_learning/online_learning.h>

class Projector
{
public:
  Projector(uint32_t seed, int projectors_per_dspace, const Object& obj);
  Eigen::VectorXf project(const Object& obj) const;
  std::string status() const;
  int getNumProjections() const;
  uint32_t getSeed() const;
  //! Takes names from the NameMapping that obj comes from.
  NameMapping2 generateNameMapping(const NameMapping& orig) const;
  
private:
  //! projectors_[i][j] is the jth projector for the ith descriptor space.
  std::vector< std::vector<Eigen::VectorXd> > projectors_;
  uint32_t seed_;
  //! Corresponds to projectors_.
  std::vector< std::vector< uint32_t > > seeds_;

  static double sampleFromGaussian(std::tr1::mt19937& mersenne_twister, double stdev);
  Eigen::VectorXd getProjector(uint32_t seed, int num_dim);
};

#endif // PROJECTOR_H
