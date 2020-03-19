#include <online_learning/schedulers.h>

using namespace std;

namespace odontomachus { 

  LearningRateScheduler* getScheduler(int scheduler_id)
  {
    LearningRateScheduler* scheduler = NULL;
    switch(scheduler_id) {
    case 0:
      scheduler = new ConstantScheduler(1.0);
      break;
    case 1:
      scheduler = new LinearScheduler(1.0);
      break;
    case 2:
      scheduler = new SqrtScheduler(1.0);
      break;
    default:
      ROS_FATAL_STREAM("Requested scheduler id " << scheduler_id << " does not exist.");
    }
    return scheduler;
  }

  LearningRateScheduler::LearningRateScheduler() :
    num_instances_(0)
  {
  }
  
  void LearningRateScheduler::incrementInstances()
  {
    ++num_instances_;
  }
  
  ConstantScheduler::ConstantScheduler(double eta_0) :
    LearningRateScheduler(),
    eta_0_(eta_0)
  {
  }

  double ConstantScheduler::getLearningRate() const
  {
    return eta_0_;
  }
  
  LinearScheduler::LinearScheduler(double eta_0) :
    LearningRateScheduler(),
    eta_0_(eta_0)
  {
  }

  double LinearScheduler::getLearningRate() const
  {
    ROS_ASSERT(fabs(num_instances_) > 1e-6);
    return eta_0_ / num_instances_;
  }

  SqrtScheduler::SqrtScheduler(double eta_0) :
    LearningRateScheduler(),
    eta_0_(eta_0)
  {
  }

  double SqrtScheduler::getLearningRate() const
  {
    ROS_ASSERT(fabs(num_instances_) > 1e-6);
    return eta_0_ / sqrt(num_instances_);
  }


} // namespace odontomachus
