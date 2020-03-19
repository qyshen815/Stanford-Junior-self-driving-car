#ifndef LEARNING_RATE_SCHEDULERS_H
#define LEARNING_RATE_SCHEDULERS_H

#include <math.h>
#include <stddef.h>
#include <ros/console.h>
#include <ros/assert.h>

namespace odontomachus {

  class LearningRateScheduler
  {
  public:
    LearningRateScheduler();
    virtual ~LearningRateScheduler() {};
    virtual double getLearningRate() const = 0;
    void incrementInstances();

  protected:
    double num_instances_;
  };

  class ConstantScheduler : public LearningRateScheduler
  {
  public:
    double eta_0_;
    
    ConstantScheduler(double eta_0);
    double getLearningRate() const;
  };

  class LinearScheduler : public LearningRateScheduler
  {
  public:
    double eta_0_;
    
    LinearScheduler(double eta_0);
    double getLearningRate() const;
  };

  class SqrtScheduler : public LearningRateScheduler
  {
  public:
    double eta_0_;
    
    SqrtScheduler(double eta_0);
    double getLearningRate() const;
  };

  LearningRateScheduler* getScheduler(int scheduler_id);

} // namespace odontomachus
  
#endif // LEARNING_RATE_SCHEDULERS_H
