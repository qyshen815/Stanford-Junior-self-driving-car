#ifndef MULTIBOOSTER_SUPPORT2_H
#define MULTIBOOSTER_SUPPORT2_H

#include "perception_types.h"
#include <track_manager.h>
#include <fstream>

#include <algorithm>
#include <multibooster/multibooster.h>
#include <cluster_descriptors/cluster_descriptors.h>
#include <sensor_msgs/PointCloud.h>
#include <point_cloud_mapping/kdtree/kdtree.h>
#include <point_cloud_mapping/kdtree/kdtree_ann.h>
#include <image_descriptor_nodes/image_descriptor_nodes.h>



void deserializeMatrix(std::istream& is, Eigen::MatrixXf* target);
void deserializeMatrix(std::string filename, Eigen::MatrixXf* target);
std::string serializeMatrix(const Eigen::MatrixXf& mat);
void serializeMatrixASCII(const Eigen::MatrixXf& mat, std::string filename);
void serializeMatrix(const Eigen::MatrixXf& mat, std::string filename);
std::string printMatrix(const std::string& filename);

void deserializeMatrix(std::istream& is, Eigen::MatrixXd* target);
void deserializeMatrix(std::string filename, Eigen::MatrixXd* target);
std::string serializeMatrix(const Eigen::MatrixXd& mat);
void serializeMatrixASCII(const Eigen::MatrixXd& mat, std::string filename);
void serializeMatrix(const Eigen::MatrixXd& mat, std::string filename);

std::string serializeVector(const Eigen::VectorXd& mat);
void deserializeVector(std::istream& is, Eigen::VectorXd* target);
void serializeVector(const Eigen::VectorXd& mat, std::string filename);
void deserializeVector(std::string filename, Eigen::VectorXd* target);
std::string printVector(const std::string& filename);
void serializeVectorASCII(const Eigen::VectorXd& vec, std::string filename);

//! Returns a matrix with columns of track responses, one column per track.
void computeTrackResponses(MultiBooster* mb,
			   const track_manager::TrackManager& tm,
			   std::vector<std::string>* classifications,
			   Eigen::MatrixXf* track_responses);

//! Returns a MultiBoosterDataset with the data from the TrackManager.
//! You must deallocate the MultiBoosterDataset yourself.
MultiBoosterDataset* computeDataset(const track_manager::TrackManager& tm);

//! 0 and 1 are one box, 2 and 3 are the other.
double computeSymmetricOutsideBoxDistance(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3);
//! 0 and 1 are the base, 2 and 3 are the new.
double computeOutsideBoxDistance(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3);
//! x0 and x1 are the base, x2 and x3 are the new.
double compute1DOutsideBoxDistance(double x0, double x1, double x2, double x3);
//! x0 < x1, y0 < y1.
void computeXYBoundingBox(const sensor_msgs::PointCloud& pc, double* x0, double* y0, double* x1, double* y1);
double computeDeltaXYFeature2(const track_manager::Track& tr, int idx, int history);
void computeCentroid(const sensor_msgs::PointCloud& pc, double* x, double* y, double* z);
void getAngles(const sensor_msgs::PointCloud& pc, double* min_angle, double* max_angle);
double computeDeltaXYFeature(const track_manager::Track& tr, int idx, int history, int length);
double computeAngleFeature(const track_manager::Track& tr, size_t idx);
double computeAngleToCentroid(const sensor_msgs::PointCloud& pc);
double computeAngleFeature2(const track_manager::Track& tr, int idx, int history_size);
void centerPointCloud(sensor_msgs::PointCloud* pc);
double computeICPDistance(sensor_msgs::PointCloud pc1, sensor_msgs::PointCloud pc2);
double computeICPDistance(cloud_kdtree::KdTree* kdt, sensor_msgs::PointCloud pc);

std::vector<std::string> getClassNames();
std::vector<std::string> getDescriptorNames();

void trackToEigen(const track_manager::Track& track,
		  std::vector< boost::shared_ptr<Eigen::MatrixXf> >* clouds,
		  std::vector< boost::shared_ptr<Eigen::VectorXf> >* intensities);  

void rosToEigen(const sensor_msgs::PointCloud &cloud,
		boost::shared_ptr<Eigen::MatrixXf>* points,
		boost::shared_ptr<Eigen::VectorXf>* intensities);

void dgcToEigen(const std::vector<point3d_t>& points,
		boost::shared_ptr<Eigen::MatrixXf>* cloud,
		boost::shared_ptr<Eigen::VectorXf>* intensities);

boost::shared_ptr<sensor_msgs::PointCloud> dgcToRos(const std::vector<point3d_t>& dgc);

void
generateDefaultDescriptorPipeline(std::vector< boost::shared_ptr<pipeline::ComputeNode> >* all_nodes);


//! Core function that determines what descriptors we use.
std::vector< boost::shared_ptr<pipeline::ComputeNode> > generateDescriptorPipeline();
std::vector< boost::shared_ptr<pipeline::ComputeNode> > generateDefaultDescriptorPipeline();
std::vector< boost::shared_ptr<pipeline::ComputeNode> > generateExperimentalDescriptorPipeline(boost::shared_ptr<PointCloudProducer> entry);
std::vector< boost::shared_ptr<pipeline::ComputeNode> > generateDescriptorPipelineBigDog(boost::shared_ptr<PointCloudProducer> entry);
				

class CachedClassifierPipeline
{
public:
  pipeline::Pipeline pipeline_;
  std::vector< boost::shared_ptr<DirectMultiBoosterNode> > classifier_nodes_;
  //! Don't set this directly.  Use setClassifier.
  MultiBooster* multibooster_;
  
  //! ClassifierPipeline does not delete mb when deallocated.
  CachedClassifierPipeline(MultiBooster* mb, int num_threads);
  std::vector<Eigen::VectorXf> classify(const std::vector<Object*>& descriptors);
  void classify(const std::vector<Object*>& descriptors, int* predicted_class_id);
  void setDebug(bool debug);
  void setClassifier(MultiBooster* multibooster);
};

class ClassifierPipeline
{
 public:
  pipeline::Pipeline pipeline_;
  std::vector< boost::shared_ptr<PointCloudEntryPoint> > input_orienters_;
  std::vector< boost::shared_ptr<MultiBoosterNode> > output_classifiers_;
  //! Don't set this directly.  Use setClassifier.
  MultiBooster* multibooster_;
  
  //! ClassifierPipeline does not delete mb when deallocated.
  ClassifierPipeline(MultiBooster* mb, int num_threads);
  std::vector<Eigen::VectorXf>
    classify(std::vector< boost::shared_ptr<Eigen::MatrixXf> > clouds,
	     std::vector< boost::shared_ptr<Eigen::VectorXf> > intensities);
  std::vector<Eigen::VectorXf> classify(boost::shared_ptr<track_manager::Track> track);
  std::vector<Eigen::VectorXf> classify(const track_manager::Track& track);
  Eigen::VectorXf classify(const track_manager::Frame& frame, std::string* label);
  std::vector<Eigen::VectorXf> classify(std::vector<Object*> descriptors);
  void setDebug(bool debug);
  void setClassifier(MultiBooster* multibooster);
};

class DescriptorPipeline
{
 public:
  pipeline::Pipeline pipeline_;
  std::vector< boost::shared_ptr<PointCloudEntryPoint> > input_orienters_;
  std::vector< boost::shared_ptr<MultiBoosterObjectConstructor> > output_constructors_;
  
  DescriptorPipeline(int num_threads);
  //! Returns the number of bytes in the descriptors for a single object.
  int getNumBytes();
  

  std::vector<Object*> computeDescriptors(const track_manager::Track& track);

  std::vector<Object*>
    computeDescriptors(std::vector< boost::shared_ptr<Eigen::MatrixXf> > clouds,
		       std::vector< boost::shared_ptr<Eigen::VectorXf> > intensities);
		       
  Object* computeDescriptors(boost::shared_ptr<Eigen::MatrixXf> cloud,
			     boost::shared_ptr<Eigen::VectorXf> intensity);

  NameMapping getDescriptorMap() const;
};

#endif //MULTIBOOSTER_SUPPORT2_H
