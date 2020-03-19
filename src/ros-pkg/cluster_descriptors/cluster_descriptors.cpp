#include <cluster_descriptors/cluster_descriptors.h>

using namespace std;
using namespace pipeline;
using namespace Eigen;
using boost::shared_ptr;


/*************************************************************
* PlaneFittingCloudOrienterOLD
**************************************************************/

PlaneFittingCloudOrienterOLD::PlaneFittingCloudOrienterOLD(int num_iterations, float tolerance) :
  PointCloudInterface(),
  tolerance_(tolerance),
  num_iterations_(num_iterations)
{
  assert(!input_cloud_);
  assert(!input_intensity_);
}

void PlaneFittingCloudOrienterOLD::setInputCloud(boost::shared_ptr<Eigen::MatrixXf> cloud) {
  input_cloud_ = cloud;
}

void PlaneFittingCloudOrienterOLD::setInputIntensity(boost::shared_ptr<Eigen::VectorXf> intensity) {
  input_intensity_ = intensity;
}

shared_ptr<MatrixXf> PlaneFittingCloudOrienterOLD::getOutputCloud() const {
  return output_cloud_;
}

shared_ptr<VectorXf> PlaneFittingCloudOrienterOLD::getOutputIntensity() const {
  return input_intensity_;
}

string PlaneFittingCloudOrienterOLD::_getName() const {
  ostringstream oss;
  oss << "PlaneFittingCloudOrienterOLD_tolerance" << tolerance_ << "_iters" << num_iterations_;
  return oss.str();
}

void PlaneFittingCloudOrienterOLD::_flush() {
  output_cloud_.reset();
  input_cloud_.reset();
  input_intensity_.reset();
}

VectorXf PlaneFittingCloudOrienterOLD::fitPlane(const MatrixXf& cloud) const {
  assert(cloud.rows() > 0);
  assert(cloud.cols() == 3);
  
  int max_num_fit = 0;
  VectorXf best_plane = VectorXf::Zero(2);
  for(int i = 0; i < num_iterations_; ++i) {
    // -- Get two random points.
    VectorXf p(2);
    int idx0 = rand() % input_cloud_->rows();
    p(0) = input_cloud_->coeff(idx0, 0);
    p(1) = input_cloud_->coeff(idx0, 1);

    VectorXf q(2);
    int idx1 = rand() % input_cloud_->rows();
    q(0) = input_cloud_->coeff(idx1, 0);
    q(1) = input_cloud_->coeff(idx1, 1);

    if(p(0) == q(0) && p(1) == q(1))
      continue;
    
    // -- Fit a line a'x = b.
    VectorXf a(2);
    a(0) = 1;
    if(p(1) - q(1) == 0)
      continue;
    else
      a(1) = -(p(0) - q(0)) / (p(1) - q(1));
    a.normalize();
    float b = a.dot(p);
    assert(fabs(a.norm() - 1) < 1e-3);
    assert(fabs(a.dot(p - q)) < 1e-3);
    assert(fabs(a.dot(q) - b) < 1e-3);

    // -- Count how many other points are close to the line.
    int num_fit = 0;
    for(int i = 0; i < input_cloud_->rows(); ++i) {
      float adotx = a.coeff(0) * input_cloud_->coeff(i, 0) + a.coeff(1) * input_cloud_->coeff(i, 1);
      if(fabs(adotx - b) <= tolerance_)
	++num_fit;
    }

    if(num_fit > max_num_fit) { 
      max_num_fit = num_fit;
      best_plane = a;
    }
  }
  return best_plane;
}

void PlaneFittingCloudOrienterOLD::_compute() {
  assert(input_cloud_);
  assert(input_intensity_);
  assert(!output_cloud_);

  // -- Fit a plane.
  VectorXf a = fitPlane(*input_cloud_);
  
  // -- Rotate the points so that the direction of the best plane is the x axis.
  //assert(fabs(a.norm() - 1) < 1e-4);
  double theta = M_PI/2. - atan2(a(1), a(0));
  MatrixXf rotation = MatrixXf::Identity(3, 3);
  rotation(0,0) = cos(theta);
  rotation(1,1) = cos(theta);
  rotation(0,1) = sin(theta);
  rotation(1,0) = -sin(theta);

  output_cloud_ = shared_ptr<MatrixXf>(new MatrixXf());
  *output_cloud_ = *input_cloud_ * rotation;

  VectorXf foo = fitPlane(*output_cloud_);
  //cout << "New plane: " << foo.transpose() << endl;
  
  // -- Subtract off the mean of the points.
  MatrixXf& points = *output_cloud_;
  VectorXf pt_mean = points.colwise().sum() / (float)points.rows();
  for(int i=0; i<points.rows(); ++i)
    points.row(i) -= pt_mean.transpose();

}


/*************************************************************
* CloudOrienter
**************************************************************/

CloudOrienter::CloudOrienter() :
  PointCloudInterface()
{
  assert(!input_cloud_);
  assert(!input_intensities_);
  assert(!output_cloud_);
}

string CloudOrienter::_getName() const {
  return string("CloudOrienter");
}

void CloudOrienter::_flush() {
  input_cloud_.reset();
  input_intensities_.reset();
  output_cloud_.reset();
}

void CloudOrienter::setInputCloud(boost::shared_ptr<Eigen::MatrixXf> cloud) {
  input_cloud_ = cloud;
}

void CloudOrienter::setInputIntensity(boost::shared_ptr<Eigen::VectorXf> intensity) {
  input_intensities_ = intensity;
}

shared_ptr<MatrixXf> CloudOrienter::getOutputCloud() const {
  return output_cloud_;
}

shared_ptr<VectorXf> CloudOrienter::getOutputIntensity() const {
  return input_intensities_;
}


void CloudOrienter::_compute() {
  assert(input_cloud_);
  assert(input_intensities_);
  assert(!output_cloud_);
  //cout << input_intensities_->rows() << " " << input_cloud_->rows() << endl;
  assert(input_cloud_->rows() == input_intensities_->rows());
  assert(input_cloud_->rows() > 2);
  
  // -- Subtract off the mean of the points.
  MatrixXf& points = *input_cloud_;
  VectorXf pt_mean = points.colwise().sum() / (float)points.rows();
  for(int i=0; i<points.rows(); ++i)
    points.row(i) -= pt_mean.transpose();

  // -- Flatten to z == 0.
  MatrixXf X = points;
  X.col(2) = VectorXf::Zero(X.rows());
  MatrixXf Xt = X.transpose();
  
  // -- Find the long axis.
  // Start with a random vector.
  VectorXf pc = VectorXf::Zero(3);
  pc(0) = 1; //Chosen by fair dice roll.
  pc(1) = 1;
  pc.normalize();
  
  // Power method.
  VectorXf prev = pc;
  double thresh = 1e-4;
  int ctr = 0;
  while(true) {
    prev = pc;
    pc =  Xt * (X * pc);
    pc.normalize();
    ctr++;
    if((pc - prev).norm() < thresh)
      break;
    // -- In some degenerate cases, it is possible for the vector
    //    to never settle down to the first PC.
    if(ctr > 100)
      break;
    
  }
  assert(abs(pc(2)) < 1e-4);
  
  // -- Find the short axis.
  VectorXf shrt = VectorXf::Zero(3);
  shrt(1) = -pc(0);
  shrt(0) = pc(1);
  assert(abs(shrt.norm() - 1) < 1e-4);
  assert(abs(shrt.dot(pc)) < 1e-4);
  
  // -- Build the basis of normalized coordinates.
  MatrixXf basis = MatrixXf::Zero(3,3);
  basis.col(0) = pc;
  basis.col(1) = shrt;
  basis(2,2) = 1.0;
  assert(abs(basis.col(0).dot(basis.col(1))) < 1e-4);
  assert(abs(basis.col(0).norm() - 1) < 1e-4);
  assert(abs(basis.col(1).norm() - 1) < 1e-4);
  assert(abs(basis.col(2).norm() - 1) < 1e-4);

  // -- Rotate and set the output_cloud_.
  output_cloud_ = shared_ptr<MatrixXf>(new MatrixXf);
  *output_cloud_ = points * basis;
  assert(output_cloud_->rows() == input_cloud_->rows());
}


/*************************************************************
* HoughCloudOrienter
**************************************************************/

HoughCloudOrienter::HoughCloudOrienter() :
  CloudOrienter()
{

}

string HoughCloudOrienter::_getName() const {
  return string("CloudOrienter");
}

void HoughCloudOrienter::_compute() {
  assert(input_cloud_);
  assert(input_intensities_);
  assert(!output_cloud_);
  //cout << input_intensities_->rows() << " " << input_cloud_->rows() << endl;
  assert(input_cloud_->rows() == input_intensities_->rows());
  assert(input_cloud_->rows() > 2);

  // -- Subtract off the mean of the points.
  MatrixXf& points = *input_cloud_;
  VectorXf pt_mean = points.colwise().sum() / (float)points.rows();
  for(int i=0; i<points.rows(); ++i)
    points.row(i) -= pt_mean.transpose();

  // -- Find the principal axis.
  static const int num_bins = 12;
  static const int max_samples = 100;
  static unsigned int count[num_bins];
  static double angle_total[num_bins];

  for (int i=0; i < num_bins; i++) {
    count[i] = 0;
    angle_total[i] = 0;
  }

  int num_points = points.rows();
  int num_samples = 0;
  unsigned int max_count = 0;
  int max_index = 0;
  while (num_samples < max_samples) {
    int ix = rand() % num_points;
    int iy = rand() % num_points;
    while (ix == iy)
      iy = rand() % num_points;

    VectorXf p1 = points.row(ix);
    VectorXf p2 = points.row(iy);
    double dy = (p1(1) - p2(1));
    double dx = (p1(0) - p2(0));
    if ((fabs(dy) < 0.001) && ( fabs(dx) < 0.001))
      continue;
    double y = atan2(dy, dx);

    // wrap into range
    if (y < 0) y += M_PI;
    if (y >= M_PI_2) y -= M_PI_2;

    int idx = (num_bins * y / M_PI_2);
    if (idx >= num_bins) {
      idx = 0;
      y = 0.0;
    }
    angle_total[idx] += y;
    count[idx]++;
    if (count[idx] > max_count) {
      max_count = count[idx];
      max_index = idx;
    }

    num_samples++;
  }
  double angle = angle_total[max_index] / max_count;

  VectorXf pc = VectorXf::Zero(3);
  pc(0) = sin(angle);
  pc(1) = cos(angle);
  pc(2) = 0.0;

  assert(abs(pc(2)) < 1e-4);

  // -- Find the short axis.
  VectorXf shrt = VectorXf::Zero(3);
  shrt(1) = -pc(0);
  shrt(0) = pc(1);
  assert(abs(shrt.norm() - 1) < 1e-4);
  assert(abs(shrt.dot(pc)) < 1e-4);

  // -- Build the basis of normalized coordinates.
  MatrixXf basis = MatrixXf::Zero(3,3);
  basis.col(0) = pc;
  basis.col(1) = shrt;
  basis(2,2) = 1.0;
  assert(abs(basis.col(0).dot(basis.col(1))) < 1e-4);
  assert(abs(basis.col(0).norm() - 1) < 1e-4);
  assert(abs(basis.col(1).norm() - 1) < 1e-4);
  assert(abs(basis.col(2).norm() - 1) < 1e-4);

  // -- Rotate and set the output_cloud_.
  output_cloud_ = shared_ptr<MatrixXf>(new MatrixXf);
  *output_cloud_ = points * basis;
  assert(output_cloud_->rows() == input_cloud_->rows());
}


/*************************************************************
* CloudSpinner
**************************************************************/

CloudSpinner::CloudSpinner(shared_ptr<PointCloudProducer> orienter) :
  ComputeNode(),
  orienter_(orienter)
{
  registerInput(orienter_);
}

string CloudSpinner::_getName() const {
  return string("CloudSpinner");
}

void CloudSpinner::_flush() {
  spin_coords_.reset();
}

void CloudSpinner::_compute() {
  assert(orienter_->getPointCloud());
  MatrixXf& xyz = *orienter_->getPointCloud();

  spin_coords_ = shared_ptr<MatrixXf>(new MatrixXf(xyz.rows(), 2));
  for(int i = 0; i < xyz.rows(); ++i) {
    spin_coords_->coeffRef(i, 0) = sqrt(xyz(i, 0) * xyz(i, 0) + xyz(i, 1) * xyz(i, 1));
    spin_coords_->coeffRef(i, 1) = xyz(i, 2);
  }

  // -- Check that they are centered.  (They are.)
  //   cout << "mean of beta: " << spin_coords_->col(1).sum() / (float)spin_coords_->rows() << endl;
//   VectorXf mean = xyz.colwise().sum() / (float)xyz.rows();
//   cout << "mean of alpha in xyz: " << sqrt(mean(0) * mean(0) + mean(1) * mean(1)) << endl;
}

/*************************************************************
* SpinImage
**************************************************************/
SpinImage::SpinImage(boost::shared_ptr<CloudSpinner> spinner, float pixels_per_meter, int num_rows, int num_cols) :
  DescriptorNode(),
  spinner_(spinner),
  pixels_per_meter_(pixels_per_meter),
  num_rows_(num_rows),
  num_cols_(num_cols),
  ipl_(NULL)
{
  registerInput(spinner);
  ipl_ = cvCreateImage(cvSize(num_cols_, num_rows_), IPL_DEPTH_32F, 1);
}

int SpinImage::getDescriptorLength() const {
  return num_rows_ * num_cols_;
}

shared_ptr<VectorXf> SpinImage::_getDescriptor() const {
  return vectorized_spin_image_;
}

SpinImage::~SpinImage() {
  if(ipl_)
    cvReleaseImage(&ipl_);
}

void SpinImage::_flush() {
  vectorized_spin_image_.reset();
}

void SpinImage::_compute() {
  assert(spinner_->spin_coords_);
  assert(spinner_->spin_coords_->rows() > 0);
  MatrixXf& spun = *spinner_->spin_coords_;

  vectorized_spin_image_ = shared_ptr<VectorXf>(new VectorXf());
  *vectorized_spin_image_ = VectorXf::Zero(num_rows_ * num_cols_);
  cvZero(ipl_);
  for(int i = 0; i < spun.rows(); ++i) {
    // -- Compute the projection.
    int alpha = spun(i, 0) * pixels_per_meter_;
    int beta = -(spun(i, 1) - (float)num_rows_  / pixels_per_meter_ / 2.) * pixels_per_meter_;
    if(alpha < 0 || alpha >= num_cols_)
      continue;
    if(beta < 0 || beta >= num_rows_)
      continue;

    // -- Fill the outputs.
    ((float*)(ipl_->imageData + beta * ipl_->widthStep))[alpha]++;

    int idx = beta * num_cols_ + alpha;
    assert(idx < num_rows_ * num_cols_);
    assert(idx >= 0);
    ++vectorized_spin_image_->coeffRef(idx);
  }

  // -- Check that the data is correct.  TODO: remove this.
  for(int y = 0; y < ipl_->height; ++y) { 
    float* ptr = (float*)(ipl_->imageData + y * ipl_->widthStep);
    for(int x = 0; x < ipl_->width; ++x, ++ptr) {
      int idx = y * num_cols_ + x;
      assert(*ptr == vectorized_spin_image_->coeffRef(idx));
    }
  }
}

void SpinImage::_display() const {

  IplImage* vis = cvCloneImage(ipl_);
  float maxval = 0;
  for(int y = 0; y < ipl_->height; ++y) { 
    float* ptr = (float*)(ipl_->imageData + y * ipl_->widthStep);
    for(int x = 0; x < ipl_->width; ++x, ++ptr) {
      if(*ptr > maxval)
	maxval = *ptr;
    }
  }
  cvConvertScale(vis, vis, 1. / maxval); //Convert from [0, maxval] to [0, 1].
  
  float scale = 10;
  IplImage* big = cvCreateImage(cvSize(((float)vis->width)*scale, ((float)vis->height)*scale),
					  vis->depth, vis->nChannels);
  cvResize(vis, big, CV_INTER_AREA);

  
  cvNamedWindow("Spin Image");
  cvShowImage("Spin Image", big);
  cvWaitKey(0);
  cvDestroyWindow("Spin Image");
  cvReleaseImage(&vis);
  cvReleaseImage(&big);
}


string SpinImage::_getName() const {
  ostringstream oss;
  oss << "SpinImage_pixelsPerMeter" << pixels_per_meter_ << "_rows" << num_rows_ << "_cols" << num_cols_;
  return oss.str();
}




/*************************************************************
* CloudProjector
**************************************************************/


CloudProjector::CloudProjector(int axis_of_projection, float pixels_per_meter, boost::shared_ptr<PointCloudProducer> orienter, int smoothing, int min_width, int min_height) :
  IplImageInterface(),
  intensity_projection_(NULL),
  depth_projection_(NULL),
  axis_of_projection_(axis_of_projection),
  pixels_per_meter_(pixels_per_meter),
  smoothing_(smoothing),
  min_width_(min_width),
  min_height_(min_height),
  orienter_(orienter)
{
  registerInput(orienter_);
}

CloudProjector::~CloudProjector() {
  _flush();
}

IplImage* CloudProjector::getImage() {
  return intensity_projection_;
}

void CloudProjector::_flush() {
  if(intensity_projection_) {
    cvReleaseImage(&intensity_projection_);
    intensity_projection_ = NULL;
  }
  if(depth_projection_) {
    cvReleaseImage(&depth_projection_);
    depth_projection_ = NULL;
  }
}


std::string CloudProjector::_getName() const {
  ostringstream oss;
  oss << "CloudProjector_axis" << axis_of_projection_ << "_ppm" << pixels_per_meter_
      << "_smoothing" << smoothing_ << "_min_width" << min_width_ << "_min_height" << min_height_;
  return oss.str();
}

void CloudProjector::_display() const {
  float scale = 10;
  IplImage* intensity_big = cvCreateImage(cvSize(((float)intensity_projection_->width)*scale, ((float)intensity_projection_->height)*scale),
					  intensity_projection_->depth, intensity_projection_->nChannels);
  cvResize(intensity_projection_, intensity_big, CV_INTER_AREA);

  IplImage* depth_big = cvCreateImage(cvSize(((float)depth_projection_->width)*scale, ((float)depth_projection_->height)*scale),
				      depth_projection_->depth, depth_projection_->nChannels);
  cvResize(depth_projection_, depth_big, CV_INTER_AREA);


  string intensity_name = "Intensity from " + getFullName();
  string depth_name = "Depth from " + getFullName();
  
  cvNamedWindow(intensity_name.c_str());
  //cvNamedWindow(depth_name.c_str());
  cvShowImage(intensity_name.c_str(), intensity_big);
  //cvShowImage(depth_name.c_str(), depth_big);
  cvWaitKey();
  cvDestroyWindow(intensity_name.c_str());
  //cvDestroyWindow(depth_name.c_str());
  cvReleaseImage(&intensity_big);
  cvReleaseImage(&depth_big);
}

void CloudProjector::_compute() {
  assert(orienter_);
  assert(orienter_->getPointCloud());
  assert(!depth_projection_);
  assert(!intensity_projection_);

  MatrixXf& oriented = *orienter_->getPointCloud();
  VectorXf& intensities = *orienter_->getIntensity();
  
  // -- Get a copy of the projected points.
  MatrixXf projected(oriented.rows(), 2);
  int c=0;
  for(int i=0; i<3; ++i) {
    if(i == axis_of_projection_)
      continue;
    projected.col(c) = oriented.col(i);
    ++c;
  }
  
  // -- Transform into pixel units.  projected is currently in meters, centered at 0.
  //projected *= pixels_per_meter_;
  for(int i=0; i<projected.rows(); ++i) {
    projected(i, 0) *= pixels_per_meter_;
    projected(i, 1) *= pixels_per_meter_;
  }
  
  
  // -- Find min and max of u and v.  TODO: noise sensitivity?
  // u is the col number in the image plane, v is the row number.
  float min_v = FLT_MAX;
  float min_u = FLT_MAX;
  float max_v = -FLT_MAX;
  float max_u = -FLT_MAX;
  for(int i=0; i<projected.rows(); ++i) {
    float u = projected(i, 0);
    float v = projected(i, 1);
    if(u < min_u)
      min_u = u;
    if(u > max_u)
      max_u = u;
    if(v < min_v)
      min_v = v;
    if(v > max_v)
      max_v = v;
  }

  // -- Translate to coordinate system where (0,0) is the upper right of the image.
  for(int i=0; i<projected.rows(); ++i) {
    projected(i, 0) -= min_u;
    projected(i, 1) = max_v - projected(i, 1);
  }
  
  // -- Get the max depth.
  float max_depth = -FLT_MAX;
  float min_depth = FLT_MAX;
  for(int i=0; i<oriented.rows(); ++i) {
    if(oriented(i, axis_of_projection_) > max_depth)
      max_depth = oriented(i, axis_of_projection_);
    if(oriented(i, axis_of_projection_) < min_depth)
      min_depth = oriented(i, axis_of_projection_);
  }

  // -- Compute the normalized depths.  Depths are between 0 and 1, with 1 meaning closest and 0 meaning furthest.
  VectorXf depths = oriented.col(axis_of_projection_);
  if(axis_of_projection_ == 1)
    depths = -depths;
  depths = (depths.array() - depths.minCoeff()).matrix();
  depths = depths / depths.maxCoeff();
  
  
  // -- Fill the IplImages.
  assert(sizeof(float) == 4);
  CvSize size = cvSize(ceil(max_u - min_u) + 1, ceil(max_v - min_v) + 1);
  float pad_width = 0;
  if(min_width_ > 0 && size.width < min_width_) { 
    pad_width = (float)(min_width_ - size.width) / 2.;
    size.width = min_width_;
  }
  float pad_height = 0;
  if(min_height_ > 0 && size.height < min_height_) {
    pad_height = (float)(min_height_ - size.height) / 2.;
    size.height = min_height_;
  }
  IplImage* acc = cvCreateImage(size, IPL_DEPTH_32F, 1);
  intensity_projection_ = cvCreateImage(size, IPL_DEPTH_32F, 1);
  depth_projection_ = cvCreateImage(size, IPL_DEPTH_32F, 1);
  cvSetZero(acc);
  cvSetZero(depth_projection_);
  cvSetZero(intensity_projection_);
  assert(intensities.rows() == projected.rows());
  for(int i=0; i<projected.rows(); ++i) {
    int row = floor(projected(i, 1) + pad_height);
    int col = floor(projected(i, 0) + pad_width);

    assert(row < size.height && row >= 0 && col < size.width && col >= 0);

    // Update accumulator.
    ((float*)(acc->imageData + row * acc->widthStep))[col]++;

    // Update intensity values.
    float val = intensities(i) / 255.0 * (3.0 / 4.0) + 0.25;
    assert(val <= 1.0 && val >= 0.0);
    ((float*)(intensity_projection_->imageData + row * intensity_projection_->widthStep))[col] += val;
    
    // Update depth values.
    ((float*)(depth_projection_->imageData + row * depth_projection_->widthStep))[col] += depths(i); //
  }   
  
  // -- Normalize by the number of points falling in each pixel.
  for(int v=0; v<acc->height; ++v) {
    float* intensity_ptr = (float*)(intensity_projection_->imageData + v * intensity_projection_->widthStep);
    float* depth_ptr = (float*)(depth_projection_->imageData + v * depth_projection_->widthStep);
    float* acc_ptr = (float*)(acc->imageData + v * acc->widthStep);
    for(int u=0; u<acc->width; ++u) {
      if(*acc_ptr == 0) {
	*intensity_ptr = 0;
	*depth_ptr = 0;
      }
      else {
	*intensity_ptr = *intensity_ptr / *acc_ptr;
	*depth_ptr = *depth_ptr / *acc_ptr;
      }

      intensity_ptr++;
      depth_ptr++;
      acc_ptr++;
    }
  }

  // -- Blur the images.  TODO: depth too?
  cvSmooth(intensity_projection_, intensity_projection_, CV_GAUSSIAN, smoothing_, smoothing_);

  
  // -- Clean up.
  cvReleaseImage(&acc);
}  



/*************************************************************
* Whitener
**************************************************************/

Whitener::Whitener(shared_ptr<DescriptorNode> node) :
  node_(node)
{
  registerInput(node);
};

int Whitener::getDescriptorLength() const {
  return node_->getDescriptorLength();
}

shared_ptr<VectorXf> Whitener::_getDescriptor() const {
  return whitened_;
}

void Whitener::_flush() {
  whitened_.reset();
}

void Whitener::_compute() {
  assert(node_->getDescriptor());
  assert(node_->getDescriptor()->rows() > 0);

  // -- Set mean to zero.
  whitened_ = shared_ptr<VectorXf>(new VectorXf());
  *whitened_ = *node_->getDescriptor();
  double mean = whitened_->sum() / (float)whitened_->rows();
  assert(!isnan(mean));
  *whitened_ = (whitened_->array() - mean).matrix();
  //assert(fabs(whitened_->sum() / (float)whitened_->rows()) < 1e-4);

  // -- Set the variance to one.
  double var = 0;
  for(int i = 0; i < whitened_->rows(); ++i)
    var += whitened_->coeffRef(i) * whitened_->coeffRef(i);
  if(var == 0)
    return;
  var /= (float)whitened_->rows();
  *whitened_ = *whitened_ * sqrt(1. / var);

  // -- Check.
  // assert(fabs(whitened_->sum() / (float)whitened_->rows()) < 1e-4);
//   var = 0;
//   for(int i = 0; i < whitened_->rows(); ++i)
//     var += whitened_->coeffRef(i) * whitened_->coeffRef(i);
//   var /= (float)whitened_->rows();
//   assert(abs(var - 1) < 1e-4);
}

string Whitener::_getName() const {
  return string("Whitener");
}

/*************************************************************
* OrientedBoundingBoxSize
**************************************************************/

OrientedBoundingBoxSize::OrientedBoundingBoxSize(shared_ptr<PointCloudProducer> orienter) :
  DescriptorNode(),
  orienter_(orienter)
{
  //TODO: How do I make registerInput see orienter as a ComputeNode?
  registerInput(orienter_);
}

void OrientedBoundingBoxSize::_compute() {
  assert(orienter_);
  assert(orienter_->getPointCloud());

  MatrixXf& cloud = *orienter_->getPointCloud();
  assert(cloud.cols() == 3);
  VectorXf min = cloud.colwise().minCoeff();
  VectorXf max = cloud.colwise().maxCoeff();
  bbox_size_ = shared_ptr<VectorXf>(new VectorXf(3));
  *bbox_size_ = max - min;
}

void OrientedBoundingBoxSize::_display() const {
  //cout << bbox_size_->transpose() << endl;
  cin.ignore();
}

int OrientedBoundingBoxSize::getDescriptorLength() const {
  return 3;
}

shared_ptr<VectorXf> OrientedBoundingBoxSize::_getDescriptor() const {
  return bbox_size_;
}



/*************************************************************
* RandomProjector
**************************************************************/

// RandomProjector::RandomProjector(const RandomProjector& rp, shared_ptr<DescriptorNode> descriptor) :
//   DescriptorNode(),
//   output_dim_(rp.output_dim_),
//   seed_(rp.seed_),
//   descriptor_(descriptor),
//   projector_(rp.projector_)
// {
//   registerInput(descriptor_);
// }

RandomProjector::RandomProjector(shared_ptr<MatrixXf> projector, shared_ptr<DescriptorNode> descriptor) :
  DescriptorNode(),
  seed_(-1),
  output_dim_(projector->rows()),
  descriptor_(descriptor),
  projector_(projector)
{
  registerInput(descriptor_);
}

RandomProjector::RandomProjector(int output_dim, int seed, boost::shared_ptr<pipeline::DescriptorNode> descriptor) :
  DescriptorNode(),
  seed_(seed),
  output_dim_(output_dim),
  descriptor_(descriptor)
{
  registerInput(descriptor_);
  projector_ = generateProjectionMatrix(descriptor_->getDescriptorLength(), output_dim_, seed_);
}


shared_ptr<MatrixXf> RandomProjector::generateProjectionMatrix(int input_dim, int output_dim, int seed) {
  shared_ptr<MatrixXf> projector(new MatrixXf(output_dim, input_dim));
  projector->setZero();
  
  // -- Each entry in the projector is drawn from the standard normal.
  // http://books.google.com/books?id=6Ewlh_lNo4sC&lpg=PP9&ots=JrJ9sqV0a5&dq=random%20projection&lr=&pg=PA2#v=twopage&q=&f=false
  std::tr1::mt19937 mersenne_twister(seed);
  for(int i=0; i<projector->rows(); ++i)
    for(int j=0; j<projector->cols(); ++j)
      projector->coeffRef(i,j) = sampleFromGaussian(mersenne_twister, 1);

  return projector;
}
 
double RandomProjector::sampleFromGaussian(std::tr1::mt19937& mersenne_twister, double stdev) {
  double sum = 0;
  for(size_t i=0; i<12; ++i) {
    sum += 2. * stdev * (double)mersenne_twister() / (double)mersenne_twister.max() - stdev;
  }
  return sum / 2.;
}

int RandomProjector::getDescriptorLength() const {
  return output_dim_;
}

boost::shared_ptr<Eigen::VectorXf> RandomProjector::_getDescriptor() const {
  return projected_descriptor_;
}

string RandomProjector::_getName() const {
  ostringstream oss;

  // -- Hash the projector matrix.
  //    This version was 32bit unsafe.
//   ostringstream moss;
//   moss << *projector_;
//   locale loc;
//   const collate<char>& coll = use_facet<collate<char> >(loc);
//   string mstr = moss.str();
//   long hash = coll.hash(mstr.data(), mstr.data() + mstr.length());

  // -- "Hash" the projection matrix.
  //    Using the sum() function does not produce the same result on 32bit.
  double hash = 0;
  for(int i = 0; i < projector_->rows(); ++i)
    for(int j = 0; j < projector_->cols(); ++j)
      hash += projector_->coeff(i,j);

  oss << "RandomProjector_outputDim" << output_dim_ << "_projectionMatrixHash" << setprecision(12) << hash;
  return oss.str();
}

void RandomProjector::_flush() {
  projected_descriptor_.reset();
}

void RandomProjector::_compute() {
  if(descriptor_->getDescriptor()) {
    projected_descriptor_ = shared_ptr<VectorXf>(new VectorXf());
    *projected_descriptor_ = (*projector_) * (*descriptor_->getDescriptor());
  }
  else
    projected_descriptor_ = shared_ptr<VectorXf>((VectorXf*)NULL);
}
 
/*************************************************************
* MultiBoosterNode
**************************************************************/

MultiBoosterNode::MultiBoosterNode(MultiBooster* booster, shared_ptr<MultiBoosterObjectConstructor> constructor) :
  booster_(booster),
  constructor_(constructor)
{
  assert(constructor_);
  assert(response_.rows() == 0);
  registerInput(constructor);
}

string MultiBoosterNode::_getName() const {
  return string("MultiBoosterNode");
}

void MultiBoosterNode::_flush() {
  response_ = VectorXf();
}

void MultiBoosterNode::_compute() {
  assert(constructor_);
  if(booster_)
    response_ = booster_->treeClassify(*constructor_->object_);
  else { 
    assert(response_.rows() == 0);
  }
}

/*************************************************************
* DirectMultiBoosterNode
**************************************************************/

DirectMultiBoosterNode::DirectMultiBoosterNode(MultiBooster* booster) :
  booster_(booster),
  did_compute_(false)
{
  assert(response_.rows() == 0);
}

string DirectMultiBoosterNode::_getName() const {
  return string("DirectMultiBoosterNode");
}

void DirectMultiBoosterNode::_flush() {
  response_ = VectorXf();
  object_ = NULL;
  assert(response_.rows() == 0);
  did_compute_ = false;
}

void DirectMultiBoosterNode::_compute() {
  assert(object_);
  assert(booster_);
  response_ = booster_->treeClassify(*object_);
  assert((size_t)response_.rows() == booster_->class_map_.size());
  assert(response_.rows() > 0);
  did_compute_ = true;
}


/*************************************************************
* MultiBoosterObjectConstructor
**************************************************************/

MultiBoosterObjectConstructor::MultiBoosterObjectConstructor(vector< shared_ptr<DescriptorNode> > descriptor_nodes) :
  descriptor_nodes_(descriptor_nodes),
  object_(NULL)
{
  for(size_t i = 0; i < descriptor_nodes_.size(); ++i) {
    registerInput(descriptor_nodes_[i]);
  }
}

int MultiBoosterObjectConstructor::getNumBytes() const {
  int num_bytes = 0;
  for(size_t i = 0; i < descriptor_nodes_.size(); ++i)
    num_bytes += descriptor_nodes_[i]->getDescriptorLength();

  return num_bytes;
}

vector<string> MultiBoosterObjectConstructor::getDescriptorNames() const {
  vector<string> names;
  for(size_t i = 0; i < descriptor_nodes_.size(); ++i)
    names.push_back(descriptor_nodes_[i]->getFullName());

  return names;
}
	
string MultiBoosterObjectConstructor::_getName() const {
  return string("MultiBoosterObjectConstructor");
}

void MultiBoosterObjectConstructor::_flush() {
  //object_.reset();
  if(object_) { 
    delete object_;
    object_ = NULL;
  }
}

void MultiBoosterObjectConstructor::_compute() {
  //object_ = shared_ptr<Object>(new Object());
  object_ = new Object();
  object_->label_ = -2; //unlabeled.
  object_->descriptors_ = vector<descriptor>(descriptor_nodes_.size());
  for(size_t i = 0; i < descriptor_nodes_.size(); ++i) {
    descriptor& desc = object_->descriptors_[i];
    if(descriptor_nodes_[i]->getDescriptor()) {
      desc.vector = new VectorXf();
      *desc.vector = *descriptor_nodes_[i]->getDescriptor(); //TODO: Make multibooster take a shared_ptr.
      desc.length_squared = desc.vector->dot(*desc.vector);
    }
    else { 
      desc.vector = NULL;
      desc.length_squared = -1;
    }
  }
}


/*************************************************************
* PointCloudEntryPoint
**************************************************************/

PointCloudEntryPoint::PointCloudEntryPoint() :
  PointCloudProducer()
{
}

shared_ptr<MatrixXf> PointCloudEntryPoint::getPointCloud() const
{
  return cloud_;
}

shared_ptr<VectorXf> PointCloudEntryPoint::getIntensity() const
{
  return intensity_;
}

string PointCloudEntryPoint::_getName() const
{
  return string("PointCloudEntryPoint");
}

void PointCloudEntryPoint::_flush()
{
  cloud_.reset();
  intensity_.reset();
  
  assert(!cloud_);
  assert(!intensity_);
}

void PointCloudEntryPoint::_compute()
{
}


/*************************************************************
* PlaneFittingCloudOrienter
**************************************************************/

PlaneFittingCloudOrienter::PlaneFittingCloudOrienter(shared_ptr<PointCloudProducer> point_cloud_producer,
						       int num_iterations, float tolerance) :
  PointCloudProducer(),
  tolerance_(tolerance),
  num_iterations_(num_iterations),
  point_cloud_producer_(point_cloud_producer)
{
  assert(point_cloud_producer_);
  registerInput(point_cloud_producer_);
}

shared_ptr<MatrixXf> PlaneFittingCloudOrienter::getPointCloud() const {
  assert(output_cloud_);
  return output_cloud_;
}

shared_ptr<VectorXf> PlaneFittingCloudOrienter::getIntensity() const {
  return point_cloud_producer_->getIntensity();
}

string PlaneFittingCloudOrienter::_getName() const {
  ostringstream oss;
  oss << "PlaneFittingCloudOrienter_tolerance" << tolerance_ << "_iters" << num_iterations_;
  return oss.str();
}

void PlaneFittingCloudOrienter::_flush() {
  output_cloud_.reset();
}

VectorXf PlaneFittingCloudOrienter::fitPlane(const MatrixXf& cloud) const {
  assert(cloud.rows() > 0);
  assert(cloud.cols() == 3);
  
  int max_num_fit = 0;
  VectorXf best_plane = VectorXf::Zero(2);
  for(int i = 0; i < num_iterations_; ++i) {
    // -- Get two random points.
    VectorXf p(2);
    int idx0 = rand() % cloud.rows();
    p(0) = cloud(idx0, 0);
    p(1) = cloud(idx0, 1);

    VectorXf q(2);
    int idx1 = rand() % cloud.rows();
    q(0) = cloud(idx1, 0);
    q(1) = cloud(idx1, 1);

    if(p(0) == q(0) && p(1) == q(1))
      continue;
    
    // -- Fit a line a'x = b.
    VectorXf a(2);
    a(0) = 1;
    if(p(1) - q(1) == 0)
      continue;
    else
      a(1) = -(p(0) - q(0)) / (p(1) - q(1));
    a.normalize();
    float b = a.dot(p);
    assert(fabs(a.norm() - 1) < 1e-3);
    assert(fabs(a.dot(p - q)) < 1e-3);
    assert(fabs(a.dot(q) - b) < 1e-3);

    // -- Count how many other points are close to the line.
    int num_fit = 0;
    for(int i = 0; i < cloud.rows(); ++i) {
      float adotx = a(0) * cloud(i, 0) + a(1) * cloud(i, 1);
      if(fabs(adotx - b) <= tolerance_)
	++num_fit;
    }

    if(num_fit > max_num_fit) { 
      max_num_fit = num_fit;
      best_plane = a;
    }
  }
  return best_plane;
}

void PlaneFittingCloudOrienter::_compute() {
  assert(!output_cloud_);
  assert(point_cloud_producer_);
  boost::shared_ptr<MatrixXf> cloud = point_cloud_producer_->getPointCloud();
  assert(cloud);
  
  // -- Fit a plane.
  VectorXf a = fitPlane(*cloud);
  
  // -- Rotate the points so that the direction of the best plane is the x axis.
  //assert(fabs(a.norm() - 1) < 1e-4);
  double theta = M_PI/2. - atan2(a(1), a(0));
  MatrixXf rotation = MatrixXf::Identity(3, 3);
  rotation(0,0) = cos(theta);
  rotation(1,1) = cos(theta);
  rotation(0,1) = sin(theta);
  rotation(1,0) = -sin(theta);

  output_cloud_ = shared_ptr<MatrixXf>(new MatrixXf());
  *output_cloud_ = *cloud * rotation;

  //VectorXf foo = fitPlane(*output_cloud_);
  //cout << "New plane: " << foo.transpose() << endl;
  
  // -- Subtract off the mean of the points.
  MatrixXf& points = *output_cloud_;
  VectorXf pt_mean = points.colwise().sum() / (float)points.rows();
  for(int i=0; i<points.rows(); ++i)
    points.row(i) -= pt_mean.transpose();

}
