#ifndef LOGISTIC_H
#define LOGISTIC_H

#include <ros/console.h>
#include <ros/assert.h>
#include <unconstrained_optimization/common_functions.h>
#include <serializable/serializable.h>
#include <eigen_extensions/eigen_extensions.h>
#include <bag_of_tricks/high_res_timer.h>

/** \brief @b Logistic represents a two-class logistic regression classifier.
 *  Public fields can be set to vary the optimization process.
 */
class Logistic : public Serializable
{
public:
  double tol_;
  double alpha_;
  double beta_;
  int max_num_iters_;
  double initial_stepsize_;
  bool debug_;
  //! Can be set by hand.  Saves a copy of the data matrix.
  boost::shared_ptr<Eigen::MatrixXd> A_;
  //! Can be set by hand.  Saves a copy of the data matrix.
  boost::shared_ptr<Eigen::VectorXd> b_;
  
  Logistic();
  //! descriptors is num_descriptors x num_training_examples.
  //! Elements of labels must be -1 or +1.
  //! Sets A_ and b_, calls train().
  void train(const Eigen::MatrixXd& descriptors, const Eigen::VectorXi& labels);
  //! Uses A_ and b_ matrices directly.
  void train();
  //! Returns the log odds.
  double classify(const Eigen::VectorXd& descriptors) const;

  Eigen::VectorXd getWeights() const;
  double getIntercept() const;
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
  
private:
  Eigen::VectorXd weights_;
  double intercept_;
};

#endif // LOGISTIC_H
