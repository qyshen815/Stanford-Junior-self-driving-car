#include <eigen_extensions/random.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

using namespace std;
using boost::normal_distribution;
using boost::variate_generator;
using boost::mt19937;
using namespace Eigen;


namespace eigen_extensions
{

  void sampleSparseGaussianVector(int rows, int nnz, SparseVector<double>* vec)
  {
    assert(rows >= nnz);

    vector<int> indices(rows);
    for(size_t i = 0; i < indices.size(); ++i)
      indices[i] = i;
    random_shuffle(indices.begin(), indices.end());

    mt19937 uniform;
    normal_distribution<> normal(0.0, 1.0);
    variate_generator<mt19937&, normal_distribution<> > sampler(uniform, normal);
    *vec = SparseVector<double>(rows);
    vec->reserve(nnz);
    for(int i = 0; i < nnz; ++i)
      vec->coeffRef(indices[i]) = sampler();
  }

} // namespace

