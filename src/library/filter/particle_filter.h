/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/
#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <sla.h>

namespace dgc {

// Particle Filter Template
// T value_type
// M state size
// N number of particles
// O observation vector size
template <class T, int M, int N, int O>
class ParticleFilter {
public:
  ParticleFilter(const sla::SMat<T,M>& state_transition_model,
                 const sla::SMat<T,M>& process_noise_covariance,
                 const sla::Vec<T,M>& initial_mean,
                 const sla::SMat<T,M>& init_covariance);
  ~ParticleFilter() {};

  // predict new particle states
  void predict();

  // observe particle states by computing gauss likelihoods
  sla::Vec<T,N>& observe(sla::Mat<T,O,N> innovation, sla::SMat<T,O> innovation_covariance);

  // resample particles
  void resample();


  sla::SMat<T,M>  state_transition_model;
  sla::SMat<T,M>  process_noise_covariance;
  sla::Mat<T,M,N> particle_states;
  sla::Vec<T,N>   particle_weights;

private:
    // compute a set of gaussian likelihoods or log-likelihoods for each innovation vector
  int gaussLikelihood(const sla::Mat<T,O,N>& innovation,
                      const sla::SMat<T,O>& innovation_covariance,
                      bool log_likelihoods,
                      sla::Vec<T,N>& likelihoods);
  // performs a stratified resampling on all particles
  void stratifiedResampling(const sla::Vec<T,N>& weights);

};

#if !defined(PARTICLE_FILTER_TEMPLATE)
#include <particle_filter_template.h>
#endif

} // namespace dgc

#endif /*PARTICLE_FILTER_H_*/
