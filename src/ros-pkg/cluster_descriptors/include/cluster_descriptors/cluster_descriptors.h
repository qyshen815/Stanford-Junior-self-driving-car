#ifndef CLUSTER_DESCRIPTORS_H_
#define CLUSTER_DESCRIPTORS_H_

#include <multibooster/multibooster.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.hpp>
#include <Eigen/Eigen>
#include <pipeline/pipeline.h>
#include <tr1/random>
#include <image_descriptor_nodes/image_descriptor_nodes.h>



class PointCloudInterface : public pipeline::ComputeNode
{
 public:
 PointCloudInterface() : ComputeNode() {}
  virtual void setInputCloud(boost::shared_ptr<Eigen::MatrixXf> cloud) = 0;
  virtual void setInputIntensity(boost::shared_ptr<Eigen::VectorXf> intensity) = 0;
  virtual boost::shared_ptr<Eigen::MatrixXf> getOutputCloud() const = 0;
  virtual boost::shared_ptr<Eigen::VectorXf> getOutputIntensity() const = 0;
};

class PlaneFittingCloudOrienterOLD : public PointCloudInterface
{
 public:
  boost::shared_ptr<Eigen::MatrixXf> input_cloud_;
  boost::shared_ptr<Eigen::VectorXf> input_intensity_;
  boost::shared_ptr<Eigen::MatrixXf> output_cloud_;

  PlaneFittingCloudOrienterOLD(int num_iterations = 100, float tolerance = 0.1);
  void setInputCloud(boost::shared_ptr<Eigen::MatrixXf> cloud);
  void setInputIntensity(boost::shared_ptr<Eigen::VectorXf> intensity);
  boost::shared_ptr<Eigen::MatrixXf> getOutputCloud() const;
  boost::shared_ptr<Eigen::VectorXf> getOutputIntensity() const;
  
 private:
  //! How close to the plane a point must be to count.
  float tolerance_;
  //! How many RANSAC iterations to run.
  int num_iterations_;

  //! Run RANSAC to find the dominant line in the XY plane.
  Eigen::VectorXf fitPlane(const Eigen::MatrixXf& cloud) const;
  std::string _getName() const;
  void _flush();
  void _compute();
};

class PointCloudProducer : public pipeline::ComputeNode
{
public:
  virtual boost::shared_ptr<Eigen::MatrixXf> getPointCloud() const = 0;
  virtual boost::shared_ptr<Eigen::VectorXf> getIntensity() const = 0;
};


class PointCloudEntryPoint : public PointCloudProducer
{
public:
  boost::shared_ptr<Eigen::MatrixXf> cloud_;
  boost::shared_ptr<Eigen::VectorXf> intensity_;

  PointCloudEntryPoint();
  boost::shared_ptr<Eigen::MatrixXf> getPointCloud() const;
  boost::shared_ptr<Eigen::VectorXf> getIntensity() const;

private:
  std::string _getName() const;
  void _flush();
  void _compute();
};

class PlaneFittingCloudOrienter : public PointCloudProducer
{
 public:
  boost::shared_ptr<Eigen::MatrixXf> output_cloud_;

  PlaneFittingCloudOrienter(boost::shared_ptr<PointCloudProducer> point_cloud_producer, int num_iterations = 100, float tolerance = 0.1);
  boost::shared_ptr<Eigen::MatrixXf> getPointCloud() const;
  boost::shared_ptr<Eigen::VectorXf> getIntensity() const;
  
 private:
  //! How close to the plane a point must be to count.
  float tolerance_;
  //! How many RANSAC iterations to run.
  int num_iterations_;
  boost::shared_ptr<PointCloudProducer> point_cloud_producer_;
  
  //! Run RANSAC to find the dominant line in the XY plane.
  Eigen::VectorXf fitPlane(const Eigen::MatrixXf& cloud) const;
  std::string _getName() const;
  void _flush();
  void _compute();
};


//! Class to put a set of point clouds into canonical orientation.  TODO: add robot pose as a parameter so that the remaining ambiguity (the principal component can have a sign flip) is resolved.
class CloudOrienter : public PointCloudInterface
{
 public:
  //! Matrices are k x 3 containing the points in (x, y, z) rows.  z is up, x is the long direction, y is the short direction.
  boost::shared_ptr<Eigen::MatrixXf> input_cloud_;
  boost::shared_ptr<Eigen::VectorXf> input_intensities_;
  boost::shared_ptr<Eigen::MatrixXf> output_cloud_;
  
  CloudOrienter();
  void setInputCloud(boost::shared_ptr<Eigen::MatrixXf> cloud);
  void setInputIntensity(boost::shared_ptr<Eigen::VectorXf> intensity);
  boost::shared_ptr<Eigen::MatrixXf> getOutputCloud() const;
  boost::shared_ptr<Eigen::VectorXf> getOutputIntensity() const;
  
 protected:
  virtual std::string _getName() const;
  virtual void _flush();
  virtual void _compute();
};

//! Class to put a set of point clouds into canonical orientation.  TODO: add robot pose as a parameter so that the remaining ambiguity (the principal component can have a sign flip) is resolved.
class HoughCloudOrienter : public CloudOrienter
{
 public:
  HoughCloudOrienter();

 protected:
  virtual std::string _getName() const;
  virtual void _compute();
};

//! Class to project canonically-oriented clusters into a virtual orthographic camera image plane.
//! Depth images currently have an ambiguity as to which side they see.
class CloudProjector : public IplImageInterface
{
 public:
  IplImage* intensity_projection_;
  IplImage* depth_projection_;
  
  CloudProjector(int axis_of_projection, float pixels_per_meter, boost::shared_ptr<PointCloudProducer> orienter, int smoothing = 3, int min_width = 0, int min_height = 0);
  ~CloudProjector();
  //! Returns the intensity projection.
  IplImage* getImage();
  
 private:
  //! 0 == x, 1 == y, 2 == z, where x, y, and z are the canonical orientation defined in CloudOrienter.
  int axis_of_projection_;
  //! Resolution of the projection.
  float pixels_per_meter_;
  //! cvSmooth parameter: size of the Gaussian kernel to use.
  int smoothing_;
  //! 0 indicates no minimum size.  Usually this is good to use in conjuction with HogArray, which needs a minimum size image to work with.
  int min_width_;
  //! 0 indicates no minimum size.
  int min_height_;
  //! Object that puts the clusters in canonical orientation.
  boost::shared_ptr<PointCloudProducer> orienter_;

  std::string _getName() const;
  void _flush();
  void _compute();
  void _display() const;
};

//! Puts x,y,z coordinates into spin coordinates (alpha, beta = z) for spin image computation.
class CloudSpinner : public pipeline::ComputeNode
{
 public:
  //! orienter_->oriented_clouds_ stored in spin coordinates, (sqrt(x^2 + y^2), z) pairs.
  boost::shared_ptr<Eigen::MatrixXf> spin_coords_;

  CloudSpinner(boost::shared_ptr<PointCloudProducer> orienter);
  
 private:
  boost::shared_ptr<PointCloudProducer> orienter_;

  std::string _getName() const;
  void _flush();
  void _compute();
};

class SpinImage : public pipeline::DescriptorNode
{
 public:
  //! row-major.
  boost::shared_ptr<Eigen::VectorXf> vectorized_spin_image_;
  
  SpinImage(boost::shared_ptr<CloudSpinner> spinner, float pixels_per_meter, int num_rows, int num_cols);
  ~SpinImage();
  int getDescriptorLength() const;
  
 private:
  boost::shared_ptr<CloudSpinner> spinner_;
  float pixels_per_meter_;
  int num_rows_;
  int num_cols_;
  IplImage* ipl_;
  

  boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const;
  void _flush();
  void _compute();
  std::string _getName() const;
  void _display() const;
};

class Whitener : public pipeline::DescriptorNode
{
 public:
  Whitener(boost::shared_ptr<pipeline::DescriptorNode> node);
  int getDescriptorLength() const;
  
 private:
  boost::shared_ptr<pipeline::DescriptorNode> node_;
  boost::shared_ptr<Eigen::VectorXf> whitened_;
  
  boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const;
  void _flush();
  void _compute();
  std::string _getName() const;
};

class OrientedBoundingBoxSize : public pipeline::DescriptorNode
{
 public:
  //! x, y, z.
  boost::shared_ptr<Eigen::VectorXf> bbox_size_;

  OrientedBoundingBoxSize(boost::shared_ptr<PointCloudProducer> orienter);
  int getDescriptorLength() const;

 private:
  boost::shared_ptr<PointCloudProducer> orienter_;

  boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const;
  void _flush() {bbox_size_.reset();}
  void _compute();
  std::string _getName() const {return std::string("OrientedBoundingBoxSize");}
  void _display() const;
};

class RandomProjector : public pipeline::DescriptorNode
{
 public:
  boost::shared_ptr<Eigen::VectorXf> projected_descriptor_;

  //! Slow constructor that generates the projection matrix.
  RandomProjector(int output_dim, int seed, boost::shared_ptr<pipeline::DescriptorNode> descriptor);
  //! Fast constructor that gets the pre-generated projection matrix from another RandomProjector.
  //RandomProjector(boost::shared_ptr<Eigen::MatrixXf> projector, boost::shared_ptr<pipeline::DescriptorNode> descriptor);

  static boost::shared_ptr<Eigen::MatrixXf> generateProjectionMatrix(int input_dim, int output_dim, int seed);
  RandomProjector(boost::shared_ptr<Eigen::MatrixXf> projector, boost::shared_ptr<pipeline::DescriptorNode> descriptor);
  int getDescriptorLength() const;

 private:
  int seed_;
  int output_dim_;
  boost::shared_ptr<pipeline::DescriptorNode> descriptor_;
  boost::shared_ptr<Eigen::MatrixXf> projector_;

  static double sampleFromGaussian(std::tr1::mt19937& mersenne_twister, double stdev);
  boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const;
  std::string _getName() const;
  void _flush();
  void _compute();
};

class MultiBoosterObjectConstructor : public pipeline::ComputeNode
{
 public:
  std::vector< boost::shared_ptr<pipeline::DescriptorNode> > descriptor_nodes_;
  //boost::shared_ptr<Object> object_;
  Object* object_;
  
  MultiBoosterObjectConstructor(std::vector< boost::shared_ptr<pipeline::DescriptorNode> > descriptor_nodes);
  void display() const {std::cout << "No display functionality for " << _getName() << std::endl;}
  int getNumBytes() const;
  std::vector<std::string> getDescriptorNames() const;
  
 private:
  std::string _getName() const;
  void _flush();
  void _compute();
};

class MultiBoosterNode : public pipeline::ComputeNode
{
 public:
  //! Response vector in order of booster_->class_map_.
  //! If booster_ is null, response_ is set to a zero-length vector.
  Eigen::VectorXf response_;
  MultiBooster* booster_;
  
  //! MultiBoosterNode does not delete booster on destruction.
  MultiBoosterNode(MultiBooster* booster,
		   boost::shared_ptr<MultiBoosterObjectConstructor> obj_constructor);

 private:

  boost::shared_ptr<MultiBoosterObjectConstructor> constructor_;

  
  std::string _getName() const;
  void _flush();
  void _compute();
};

class DirectMultiBoosterNode : public pipeline::ComputeNode
{
 public:
  //! Response vector in order of booster_->class_map_.
  Eigen::VectorXf response_;
  MultiBooster* booster_;
  //! Set this by hand every time.
  Object* object_;
  //! debugging
  bool did_compute_;
  
  //! MultiBoosterNode does not delete booster on destruction.
  DirectMultiBoosterNode(MultiBooster* booster);

private:
  std::string _getName() const;
  void _flush();
  void _compute();
};


/* class DescriptorPipeline : public pipeline::Pipeline */
/* { */
/*  public: */
/*   DescriptorPipeline(int num_components, int num_threads); */
/*   //! clouds.size() <= num_components_. */
/*   void setInputs(const std::vector< boost::shared_ptr<Eigen::MatrixXf> >& clouds, */
/* 		 const std::vector< boost::shared_ptr<Eigen::VectorXf> >& intensities); */
/*   //! For the special case of a single pipeline component. */
/*   void setInput(boost::shared_ptr<Eigen::MatrixXf> cloud, */
/* 		boost::shared_ptr<Eigen::VectorXf> intensity); */
/*   std::vector<std::string> getDescriptorNames() const; */
/*   //! Returns the number of bytes in all the descriptors in a single pipeline component. */
/*   int getNumBytes() const; */
/* /\*   std::vector< std::vector< boost::shared_ptr<Eigen::VectorXf> > > *\/ */
/* /\*     computeDescriptors(boost::shared_ptr<Eigen::MatrixXf> clouds, *\/ */
/* /\* 		       boost::shared_ptr<Eigen::VectorXf> intensities); *\/ */

  
/*   std::vector< boost::shared_ptr<pipeline::DescriptorNode> > getOutputDescriptorNodes(size_t id); */
/*   std::vector< std::vector< boost::shared_ptr<pipeline::DescriptorNode> > > getOutputDescriptorNodes(); */
  
/*  private: */
/*   //! The number of pipeline components, i.e. the number of clusters to handle at once. */
/*   int num_components_; */
/*   //! Number of threads to use. */
/*   int num_threads_; */
/*   //! output_nodes_[i] is the vector of all output descriptor nodes for component i. */
/*   std::vector< std::vector< boost::shared_ptr<pipeline::DescriptorNode> > > output_nodes_; */
/*   //! cloud_orienters_[i] is the cloud orienter (only input) for component i. */
/*   std::vector< boost::shared_ptr<PointCloudInterface> > cloud_orienters_; */

/* }; */


/* void generateClusterDescriptorPipeline(std::vector< boost::shared_ptr<Eigen::MatrixXf> > clouds, */
/* 				       std::vector< boost::shared_ptr<Eigen::VectorXf> > intensities, */
/* 				       std::vector< boost::shared_ptr<pipeline::ComputeNode> >* nodes, */
/* 				       std::vector< boost::shared_ptr<pipeline::DescriptorNode> >* descriptor_nodes); */

#endif // CLUSTER_DESCRIPTORS_H_
