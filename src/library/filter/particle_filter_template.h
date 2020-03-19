/*--------------------------------------------------------------------
 * project ....: Darpa Urban Challenge 2007
 * authors ....: Team AnnieWAY
 * organization: Transregional Collaborative Research Center 28 /
 *               Cognitive Automobiles
 * creation ...: xx/xx/2007
 * revisions ..: $Id:$
---------------------------------------------------------------------*/

namespace dgc {

#define PARTICLE_FILTER_TEMPLATE

// initialise particle filter with multivariate gauss distribution
template <class T, int M, int N, int O>
ParticleFilter<T,M,N,O>::
ParticleFilter(const sla::SMat<T,M>& _state_transition_model,
               const sla::SMat<T,M>& _process_noise_covariance,
               const sla::Vec<T,M>& initial_mean,
               const sla::SMat<T,M>& init_covariance)
: state_transition_model(_state_transition_model),
  process_noise_covariance(_process_noise_covariance) {
  particle_states  = sla::Mat<T,M,N>::multivariateGauss(initial_mean, init_covariance);
}

// predict next particle state
template <class T, int M, int N, int O>
void ParticleFilter<T,M,N,O>::predict() {
  sla::Mat<T,M,N> additive_gaussian_noise;
  additive_gaussian_noise = sla::Mat<T,M,N>::multivariateGauss(sla::Vec<T,M>(T(0)), process_noise_covariance);
  particle_states = state_transition_model*particle_states+additive_gaussian_noise;
}

// observe particle states
template <class T, int M, int N, int O>
sla::Vec<T,N>& ParticleFilter<T,M,N,O>::observe(sla::Mat<T,O,N> innovation, sla::SMat<T,O> innovation_covariance) {
  // calculate gauss likelihoods
  gaussLikelihood(innovation,innovation_covariance,false,particle_weights);
  return particle_weights;
}

// resample particles
template <class T, int M, int N, int O>
void ParticleFilter<T,M,N,O>::resample() {
  // resample particles
  stratifiedResampling(particle_weights);
}

template <class T, int M, int N, int O>
void ParticleFilter<T,M,N,O>::
stratifiedResampling(const sla::Vec<T,N>& weights) {
  sla::Vec<int,N> keep;

  // normalize weights
  sla::Vec<T,N> normalized_weights = weights/weights.sum();

  // generate stratified random numbers
  sla::Vec<T,N> stratified_random = sla::Vec<T,N>::uniformRand(T(0),T(1));
  const T N_inv = T(1)/T(N);
  for(int i=0;i<N;++i) {
    stratified_random.x[i] = (T(i) + stratified_random.x[i]) * N_inv;
  }

  // cumulative sum
  sla::Vec<T,N> cumulative_weights = normalized_weights.cumsum();

  // good particles
  int j=0;
  for(int i=0;i<N;++i) {
    while(j<N && stratified_random[j]<cumulative_weights[i]) {
      keep(j)=i;
      j++;
    }
  }

  // duplicate good particles
  sla::Mat<T,M,N> particle_states_copy(particle_states);
  for(int i=0;i<N;++i) {
    if(i==keep(i)) continue;
    particle_states.setCol(i,particle_states_copy.getCol(keep(i)));
  }
}


template <class T, int M, int N, int O>
int ParticleFilter<T,M,N,O>::
gaussLikelihood(const sla::Mat<T,O,N>& innovation,
                const sla::SMat<T,O>& innovation_covariance,
                bool log_likelihoods,
                sla::Vec<T,N>& likelihoods) {
  sla::SMat<T,O> S(innovation_covariance);
  int n=S.choleskyDecomp();
  if(n==1) return 1;
  S.transpose();
  sla::Mat<T,O,O> S_inv = S.inverted();
  sla::Mat<T,O,N> normalised_innovation = S_inv*innovation;

   // normalizing term (normalizes gaussian volume to 1)
  T c = sla::pow(T(2)*PI,T(O)/T(2))*sla::abs(S.getDiag().prod());
  // calculate exponetial term
  normalised_innovation.perform(sla::sqr);
  sla::Vec<T,N> e = T(-0.5) * normalised_innovation.sum();
  // calculate likelihoods
  if(log_likelihoods) {
    likelihoods = e-log(c);
  } else {
    e.perform(exp);
    likelihoods = e/c;
  }
  return 0;
}

} // namespace dgc
