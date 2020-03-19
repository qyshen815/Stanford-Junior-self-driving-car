#ifndef TRACK_DESCRIPTORS_H_
#define TRACK_DESCRIPTORS_H_

#include <pipeline/pipeline.h>
#include <track_manager.h>
#include <cluster_descriptors/cluster_descriptors.h>
#include <multibooster_support.h>

namespace track_descriptors { 

class TrackInterface : public pipeline::ComputeNode
{
 public:
  boost::shared_ptr<track_manager::Track> track_;

 TrackInterface() : ComputeNode() {assert(!track_);}

 protected:
  std::string _getName() const;
  void _flush();
  void _compute();
  void _display() const;
};

class VelocitySmoother : public pipeline::ComputeNode
{
 public:
  boost::shared_ptr<TrackInterface> interface_;
  //! velocities_.col(i) is the velocity vector for the ith frame.
  boost::shared_ptr<Eigen::MatrixXf> velocities_;
  boost::shared_ptr<Eigen::VectorXf> magnitudes_;
  boost::shared_ptr<Eigen::VectorXd> timestamps_;
  //! Number of frames to smooth over.
  int history_;
  //! If true, use the bounding box velocity estimate.  Otherwise, use the centroid.
  bool use_bbox_;
  
  VelocitySmoother(boost::shared_ptr<TrackInterface> interface, int history, bool use_bbox);

 protected:
  std::string _getName() const;
  void _flush();
  void _compute();
  void _display() const;
  
};


class AngularVelocity : public pipeline::ComputeNode
{
 public:
  boost::shared_ptr<Eigen::VectorXf> angular_velocities_;
  
  AngularVelocity(boost::shared_ptr<VelocitySmoother> velocity_smoother, int history, double velocity_thresh);
  
 protected:
  std::string _getName() const;
  void _flush();
  void _compute();
  void _display() const;

 private:
  boost::shared_ptr<VelocitySmoother> velocity_smoother_;
  int history_;
  double velocity_thresh_;
};



class MaxAngularVelocity : public pipeline::DescriptorNode
{
 public:
  boost::shared_ptr<Eigen::VectorXf> max_angular_vel_;

  MaxAngularVelocity(boost::shared_ptr<AngularVelocity> angular_velocity);
  int getDescriptorLength() const;
  
 protected:
  std::string _getName() const;
  void _flush();
  void _compute();
  void _display() const;
  boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const;
    
 private:
  boost::shared_ptr<AngularVelocity> angular_velocity_;
};

class MeanVelocity : public pipeline::DescriptorNode
{
 public:
  boost::shared_ptr<Eigen::VectorXf> mean_vel_;

  MeanVelocity(boost::shared_ptr<VelocitySmoother> velocity_smoother);
  int getDescriptorLength() const;
  
 protected:
  std::string _getName() const;
  void _flush();
  void _compute();
  void _display() const;
  boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const;
    
 private:
  boost::shared_ptr<VelocitySmoother> velocity_smoother_;
 };

class MaxVelocity : public pipeline::DescriptorNode
{
 public:
  boost::shared_ptr<Eigen::VectorXf> max_vel_;

  MaxVelocity(boost::shared_ptr<VelocitySmoother> velocity_smoother);
  int getDescriptorLength() const;
  
 protected:
  std::string _getName() const;
  void _flush();
  void _compute();
  void _display() const;
  boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const;
    
 private:
  boost::shared_ptr<VelocitySmoother> velocity_smoother_;
};



class MaxAcceleration : public pipeline::DescriptorNode
{
 public:
  boost::shared_ptr<Eigen::VectorXf> max_acc_;

  MaxAcceleration(boost::shared_ptr<VelocitySmoother> velocity_smoother);
  int getDescriptorLength() const;
  
 protected:
  std::string _getName() const;
  void _flush();
  void _compute();
  void _display() const;
  boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const;
    
 private:
  boost::shared_ptr<VelocitySmoother> velocity_smoother_;
};
      

class NaiveTrackAccumulator : public PointCloudProducer
{
public:
  boost::shared_ptr<Eigen::MatrixXf> output_cloud_;
  boost::shared_ptr<Eigen::VectorXf> output_intensity_;

  NaiveTrackAccumulator(boost::shared_ptr<TrackInterface> interface, int max_points);
  //! Disabled, since the input to this class is a Track.  PointCloudInterface should be refactored.
  virtual void setInputCloud(boost::shared_ptr<Eigen::MatrixXf> cloud);
  //! Disabled, since the input to this class is a Track.  PointCloudInterface should be refactored.
  virtual void setInputIntensity(boost::shared_ptr<Eigen::VectorXf> intensity);

  boost::shared_ptr<Eigen::MatrixXf> getPointCloud() const;
  boost::shared_ptr<Eigen::VectorXf> getIntensity() const;
  
private:
  boost::shared_ptr<TrackInterface> interface_;
  //! If the accumulated point cloud has more than this number of points,
  //! the cloud is randomly downsampled to have this many.
  int max_points_;
  
  std::string _getName() const;
  void _flush();
  void _compute();
};
 
  
class TrackClassifierPipeline
{
 public:
  TrackClassifierPipeline(MultiBooster* booster, bool debug = false);
  Object* computeMultiBoosterObject(boost::shared_ptr<track_manager::Track> track);
  Eigen::VectorXf classify(boost::shared_ptr<track_manager::Track> track);
  std::vector<std::string> getDescriptorNames() const;
  NameMapping getClassMap() const;
  
 private:
  pipeline::Pipeline pipeline_;
  boost::shared_ptr<TrackInterface> interface_;
  boost::shared_ptr<MultiBoosterObjectConstructor> constructor_;
  boost::shared_ptr<MultiBoosterNode> classifier_;
  bool debug_;
  
  void initialize(MultiBooster* booster);
};

class CombinedClassifierPipeline
{
 public:
  int num_threads_;
  TrackClassifierPipeline* tcp_;
  ClassifierPipeline* fcp_;
  MultiBooster* frame_classifier_;
  MultiBooster* track_classifier_;
  //! weights_(0) is the track classifier coefficient, weights_(1) is the mean frame classifier coefficient.
  Eigen::VectorXd weights_;
  
  CombinedClassifierPipeline(const std::string& frame_classifier,
			     const std::string& track_classifier,
			     const Eigen::VectorXd& weights,
			     int num_threads);
  ~CombinedClassifierPipeline();
  Eigen::VectorXf classify(boost::shared_ptr<track_manager::Track> track);
  Eigen::VectorXf classify(boost::shared_ptr<track_manager::Track> track,
			   Eigen::VectorXf* track_response,
			   std::vector<Eigen::VectorXf>* frame_responses);
  void classify(boost::shared_ptr<track_manager::Track> track, std::string* label);
  NameMapping getClassMap() const;
  void setDebug(bool debug);
};
    
} // namespace track_descriptors  

#endif // TRACK_DESCRIPTORS_H_
