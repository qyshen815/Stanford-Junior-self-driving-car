#include <track_descriptors.h>

using namespace std;
using boost::shared_ptr;
using namespace Eigen;

using namespace track_manager;
using namespace pipeline;

namespace track_descriptors {

  /************************************************************
   * TrackInterface
   ************************************************************/
  
  string TrackInterface::_getName() const {
    return "TrackInterface";
  }

  void TrackInterface::_flush() {
    track_.reset();
  }

  void TrackInterface::_compute() {
  }

  void TrackInterface::_display() const {
    cout << "TrackInterface has a track with " << track_->frames_.size() << " frames." << endl;
  }

  /************************************************************
   * VelocitySmoother
   ************************************************************/
  
  VelocitySmoother::VelocitySmoother(shared_ptr<TrackInterface> interface, int history, bool use_bbox) :
    interface_(interface),
    history_(history),
    use_bbox_(use_bbox)
  {
    registerInput(interface_);
  }
  
  string VelocitySmoother::_getName() const {
    ostringstream oss;
    oss << "VelocitySmoother_history" << history_ << "_use_bbox" << use_bbox_;
    return oss.str();
  }

  void VelocitySmoother::_flush() {
    velocities_.reset();
    magnitudes_.reset();
    timestamps_.reset();
  }

  int sign(double x) {
    if(x > 0)
      return 1;
    if(x < 0)
      return -1;
    return 0;
  }

  Vector2f computeBoundingBoxVelocity(shared_ptr<Frame> frame1, shared_ptr<Frame> frame2) {
    Vector2f vel = Vector2f::Zero();
    MatrixXf mat1 = frame1->getBoundingBox();
    MatrixXf mat2 = frame2->getBoundingBox();

    double dx_left = mat1(0, 0) - mat2(0, 0);
    double dx_right = mat1(0, 1) - mat2(0, 1); 
    if(sign(dx_left) == sign(dx_right)) { 
      if(fabs(dx_left) > fabs(dx_right))
	vel(0) = dx_right;
      else
	vel(0) = dx_left;
    }

    double dy_left = mat1(1, 0) - mat2(1, 0);
    double dy_right = mat1(1, 1) - mat2(1, 1);
    if(sign(dy_left) == sign(dy_right)) { 
      if(fabs(dy_left) > fabs(dy_right))
	vel(1) = dy_right;
      else
	vel(1) = dy_left;
    }
    
    vel /= fabs(frame2->timestamp_ - frame1->timestamp_);
    return vel;
  }
  
  void VelocitySmoother::_compute() {
    Track& tr = *interface_->track_;
    int length = tr.frames_.size() - history_;
    velocities_ = shared_ptr<MatrixXf>(new MatrixXf(2, length));
    magnitudes_ = shared_ptr<VectorXf>(new VectorXf(length));
    timestamps_ = shared_ptr<VectorXd>(new VectorXd(length));

    for(size_t i = history_; i < tr.frames_.size(); ++i) {
      timestamps_->coeffRef(i - history_) = tr.frames_[i]->timestamp_;

      if(use_bbox_) { 
	velocities_->col(i - history_) = computeBoundingBoxVelocity(tr.frames_[i], tr.frames_[i - history_]);
      }
      else { 
	Vector3f current = tr.frames_[i]->getCentroid();
	Vector3f past = tr.frames_[i - history_]->getCentroid();
	double delta_t = tr.frames_[i]->timestamp_ - tr.frames_[i - history_]->timestamp_;
	velocities_->col(i - history_) = (current - past).segment(0, 2) / delta_t;
	(*magnitudes_)(i - history_) = velocities_->col(i - history_).norm();
      }

      (*magnitudes_)(i - history_) = velocities_->col(i - history_).norm();
    }
  }

  void VelocitySmoother::_display() const {
    cout << "Velocities: " << endl << *velocities_ << endl;
    cout << "Magnitudes: " << endl << magnitudes_->transpose() << endl;
    cout << "Timestamps: " << endl << timestamps_->transpose() << endl;
  }

  /************************************************************
   * AngularVelocity
   ************************************************************/

  AngularVelocity::AngularVelocity(shared_ptr<VelocitySmoother> velocity_smoother, int history, double velocity_thresh) :
    velocity_smoother_(velocity_smoother),
    history_(history),
    velocity_thresh_(velocity_thresh)
  {
    assert(history_ > 0);
    assert(velocity_thresh_ > 0.0);
    registerInput(velocity_smoother);
  }

  string AngularVelocity::_getName() const {
    ostringstream oss;
    oss << "AngularVelocity_history" << history_;
    return oss.str();
  }

  void AngularVelocity::_flush() {
    angular_velocities_.reset();
  }

  void AngularVelocity::_compute() {
    MatrixXf& vels = *velocity_smoother_->velocities_;
    VectorXf& mags = *velocity_smoother_->magnitudes_;
    VectorXd& timestamps = *velocity_smoother_->timestamps_;
    angular_velocities_ = shared_ptr<VectorXf>(new VectorXf(vels.cols() - history_));
    angular_velocities_->setZero();
    
    for(int i = history_; i < vels.cols(); ++i) {
      if(mags(i) < velocity_thresh_ || mags(i - history_) < velocity_thresh_)
	continue;
      VectorXf a = vels.col(i);
      VectorXf b = vels.col(i - history_);
      double theta = acos(a.dot(b) / (a.norm() * b.norm()));
      double delta_t = timestamps(i) - timestamps(i - history_);
      (*angular_velocities_)(i - history_) = fabs((double)theta / delta_t);
    }
  }

  void AngularVelocity::_display() const {
    cout << angular_velocities_->transpose() << endl;
  }


  /************************************************************
   * MaxAngularVelocity
   ************************************************************/
  MaxAngularVelocity::MaxAngularVelocity(shared_ptr<AngularVelocity> angular_velocity) :
    DescriptorNode(),
    angular_velocity_(angular_velocity)
  {
    registerInput(angular_velocity_);
  }

  string MaxAngularVelocity::_getName() const {
    return "MaxAngularVelocity";
  }

  void MaxAngularVelocity::_flush() {
    max_angular_vel_.reset();
  }

  void MaxAngularVelocity::_compute() {
    max_angular_vel_ = shared_ptr<VectorXf>(new VectorXf(1));
    max_angular_vel_->coeffRef(0) = angular_velocity_->angular_velocities_->maxCoeff();
  }

  void MaxAngularVelocity::_display() const {
  }

  shared_ptr<VectorXf> MaxAngularVelocity::_getDescriptor() const {
    return max_angular_vel_;
  }

  int MaxAngularVelocity::getDescriptorLength() const {
    return 1;
  }
  
  
  
  /************************************************************
   * MaxVelocity
   ************************************************************/
  
  MaxVelocity::MaxVelocity(shared_ptr<VelocitySmoother> velocity_smoother) :
    DescriptorNode(),
    velocity_smoother_(velocity_smoother)
  {
    registerInput(velocity_smoother_);
  }
  
  string MaxVelocity::_getName() const {
    return "MaxVelocity";
  }

  void MaxVelocity::_flush() {
    max_vel_.reset();
  }

  void MaxVelocity::_compute() {
    max_vel_ = shared_ptr<VectorXf>(new VectorXf(1));
    max_vel_->coeffRef(0) = velocity_smoother_->magnitudes_->maxCoeff();
  }

  void MaxVelocity::_display() const {
  }

  shared_ptr<VectorXf> MaxVelocity::_getDescriptor() const {
    return max_vel_;
  }

  int MaxVelocity::getDescriptorLength() const {
    return 1;
  }

  /************************************************************
   * MeanVelocity
   ************************************************************/
  
  MeanVelocity::MeanVelocity(shared_ptr<VelocitySmoother> velocity_smoother) :
    DescriptorNode(),
    velocity_smoother_(velocity_smoother)
  {
    registerInput(velocity_smoother_);
  }
  
  string MeanVelocity::_getName() const {
    return "MeanVelocity";
  }

  void MeanVelocity::_flush() {
    mean_vel_.reset();
  }

  void MeanVelocity::_compute() {
    mean_vel_ = shared_ptr<VectorXf>(new VectorXf(1));
    mean_vel_->coeffRef(0) = velocity_smoother_->magnitudes_->sum() / (double)velocity_smoother_->magnitudes_->rows();
  }

  void MeanVelocity::_display() const {
  }

  shared_ptr<VectorXf> MeanVelocity::_getDescriptor() const {
    return mean_vel_;
  }

  int MeanVelocity::getDescriptorLength() const {
    return 1;
  }

  

  /************************************************************
   * MaxAcceleration
   ************************************************************/

  MaxAcceleration::MaxAcceleration(shared_ptr<VelocitySmoother> velocity_smoother) :
    DescriptorNode(),
    velocity_smoother_(velocity_smoother)
  {
    registerInput(velocity_smoother_);
  }
  
  string MaxAcceleration::_getName() const {
    return "MaxAcceleration";
  }

  void MaxAcceleration::_flush() {
    max_acc_.reset();
  }

  void MaxAcceleration::_compute() {
    max_acc_ = shared_ptr<VectorXf>(new VectorXf(1));

    // -- Compute the accelerations.
    VectorXf& vels = *velocity_smoother_->magnitudes_;
    VectorXf accs(vels.rows() - 1);
    for(int i = 1; i < vels.rows(); ++i) {
      accs(i-1) = fabs(vels(i) - vels(i-1));
    }
    max_acc_->coeffRef(0) = accs.maxCoeff();
  }

  void MaxAcceleration::_display() const {
  }

  shared_ptr<VectorXf> MaxAcceleration::_getDescriptor() const {
    return max_acc_;
  }

  int MaxAcceleration::getDescriptorLength() const {
    return 1;
  }


  /************************************************************
   * TrackClassifierPipeline
   ************************************************************/
  
  TrackClassifierPipeline::TrackClassifierPipeline(MultiBooster* booster, bool debug) :
    pipeline_(),
    debug_(debug)
  {
    initialize(booster);
  }

  void TrackClassifierPipeline::initialize(MultiBooster* booster) {
    vector< shared_ptr<ComputeNode> > nodes;

    interface_ = shared_ptr<TrackInterface>(new TrackInterface());
    nodes.push_back(interface_);

    // -- Accumulated track descriptors.
    shared_ptr<NaiveTrackAccumulator> track_accumulator(new NaiveTrackAccumulator(interface_, 10000));
    assert(track_accumulator);
    nodes.push_back(track_accumulator);
    
    vector< shared_ptr<ComputeNode> > frame_descriptor_nodes = generateExperimentalDescriptorPipeline(track_accumulator);
    nodes.insert(nodes.end(), frame_descriptor_nodes.begin(), frame_descriptor_nodes.end());
    
    // -- Raw motion descriptors.
    shared_ptr<VelocitySmoother> velocity_smoother(new VelocitySmoother(interface_, 4, false));
    nodes.push_back(velocity_smoother);
    
    shared_ptr<AngularVelocity> angular_velocity(new AngularVelocity(velocity_smoother, 4, 1.0));
    nodes.push_back(angular_velocity);
    nodes.push_back(shared_ptr<MaxVelocity>(new MaxVelocity(velocity_smoother)));
    nodes.push_back(shared_ptr<MeanVelocity>(new MeanVelocity(velocity_smoother)));
    nodes.push_back(shared_ptr<MaxAcceleration>(new MaxAcceleration(velocity_smoother)));
    nodes.push_back(shared_ptr<MaxAngularVelocity>(new MaxAngularVelocity(angular_velocity)));

    // -- Bounding box velocity based descriptors.
    shared_ptr<VelocitySmoother> bbox_velocity_smoother(new VelocitySmoother(interface_, 4, true));
    nodes.push_back(bbox_velocity_smoother);
    
    shared_ptr<AngularVelocity> bbox_angular_velocity(new AngularVelocity(bbox_velocity_smoother, 4, 1.0));
    nodes.push_back(bbox_angular_velocity);
    nodes.push_back(shared_ptr<MaxVelocity>(new MaxVelocity(bbox_velocity_smoother)));
    nodes.push_back(shared_ptr<MeanVelocity>(new MeanVelocity(bbox_velocity_smoother)));
    nodes.push_back(shared_ptr<MaxAcceleration>(new MaxAcceleration(bbox_velocity_smoother)));
    nodes.push_back(shared_ptr<MaxAngularVelocity>(new MaxAngularVelocity(bbox_angular_velocity)));

    constructor_ = shared_ptr<MultiBoosterObjectConstructor>(new MultiBoosterObjectConstructor(filterNodes<DescriptorNode>(nodes)));
    nodes.push_back(constructor_);

    classifier_ = shared_ptr<MultiBoosterNode>(new MultiBoosterNode(booster, constructor_)); // If booster == NULL, this node won't do anything.
    nodes.push_back(classifier_);

    if(debug_)
      cout << "Debugging mode.." << endl;
    for(size_t i = 0; i < nodes.size(); ++i)
      nodes[i]->debug_ = debug_;

    pipeline_.setNodes(nodes);
  }

  Object* TrackClassifierPipeline::computeMultiBoosterObject(shared_ptr<Track> track) {
    interface_->track_ = track;
    pipeline_.compute();
    Object* obj = new Object(*constructor_->object_); // Deep copy.
    pipeline_.flush();
    return obj;
  }

  std::vector<std::string> TrackClassifierPipeline::getDescriptorNames() const {
    return constructor_->getDescriptorNames();
  }

  Eigen::VectorXf TrackClassifierPipeline::classify(boost::shared_ptr<track_manager::Track> track) {
    assert(classifier_->booster_);
    interface_->track_ = track;
    pipeline_.compute();
    VectorXf response = classifier_->response_;
    pipeline_.flush();
    return response;
  }

  NameMapping TrackClassifierPipeline::getClassMap() const {
    return classifier_->booster_->class_map_;
  }

  CombinedClassifierPipeline::CombinedClassifierPipeline(const std::string& frame_classifier,
							 const std::string& track_classifier,
							 const VectorXd& weights,
							 int num_threads) :
    num_threads_(num_threads),
    weights_(weights)
  {
    frame_classifier_ = new MultiBooster(frame_classifier);
    track_classifier_ = new MultiBooster(track_classifier);

    tcp_ = new TrackClassifierPipeline(track_classifier_);
    fcp_ = new ClassifierPipeline(frame_classifier_, num_threads);

    frame_classifier_->applyNewMappings(track_classifier_->class_map_, getDescriptorNames());
    track_classifier_->applyNewMappings(track_classifier_->class_map_, tcp_->getDescriptorNames());

    assert(frame_classifier_->class_map_.compare(track_classifier_->class_map_)); // Make sure the response vectors are in the same order.
  }

  CombinedClassifierPipeline::~CombinedClassifierPipeline() {
    delete tcp_;
    delete fcp_;
    delete frame_classifier_;
    delete track_classifier_;
  }


  void CombinedClassifierPipeline::classify(shared_ptr<Track> track, string* label) {
    VectorXf response = classify(track);
    int prediction = -1;
    double val = response.maxCoeff(&prediction);
    if(val <= 0)
      prediction = -1;

    if(prediction == -1)
      *label = "background";
    else
      *label = track_classifier_->class_map_.toName(prediction);
  }
  
  VectorXf CombinedClassifierPipeline::classify(shared_ptr<Track> track) {
    VectorXf track_response;
    vector<VectorXf> frame_responses;
    return classify(track, &track_response, &frame_responses);
  }
  

  VectorXf CombinedClassifierPipeline::classify(boost::shared_ptr<track_manager::Track> track,
						Eigen::VectorXf* track_response,
						std::vector<Eigen::VectorXf>* frame_responses) {
    assert(track_response);
    assert(frame_responses);

    timeval start, end;
    gettimeofday(&start, NULL);
    *track_response = tcp_->classify(track);
    gettimeofday(&end, NULL);
    cout << "Global (amortized): " << ((end.tv_sec - start.tv_sec) * 1000. + (end.tv_usec - start.tv_usec) / 1000.) / (double)track->frames_.size() << " ms." << endl;

    gettimeofday(&start, NULL);
    *frame_responses = fcp_->classify(track);
    gettimeofday(&end, NULL);
    double ms = (end.tv_sec - start.tv_sec) * 1000. + (end.tv_usec - start.tv_usec) / 1000.;
    cout << "Frame: " << ms / (double)track->frames_.size() << " ms per frame." << endl;

    VectorXf combined = VectorXf::Zero(track_response->rows());
    for(size_t i = 0; i < frame_responses->size(); ++i) {
      combined += 2.0 * (frame_responses->at(i) - frame_classifier_->prior_);
    }
    combined *= weights_(2) / (double)frame_responses->size();
    combined += 2.0 * (*track_response - track_classifier_->prior_) * weights_(1);
    combined += 2.0 * track_classifier_->prior_ * weights_(0);

    return combined;
  }

  NameMapping CombinedClassifierPipeline::getClassMap() const {
    return tcp_->getClassMap();
  }

  void CombinedClassifierPipeline::setDebug(bool debug) {
    fcp_->setDebug(debug);
  }



/************************************************************
 * NaiveTrackAccumulator
 ************************************************************/ 
NaiveTrackAccumulator::NaiveTrackAccumulator(shared_ptr<TrackInterface> interface, int max_points) :
  PointCloudProducer(),
  interface_(interface),
  max_points_(max_points)
{
  assert(!output_intensity_);
  assert(!output_cloud_);
  assert(interface_);
  assert(max_points_ > 0);
  
  registerInput(interface_);
}
  
void NaiveTrackAccumulator::setInputCloud(boost::shared_ptr<Eigen::MatrixXf> cloud)
{
  assert(false);
}
  
void NaiveTrackAccumulator::setInputIntensity(boost::shared_ptr<Eigen::VectorXf> intensity)
{
  assert(false);
}
  
shared_ptr<MatrixXf> NaiveTrackAccumulator::getPointCloud() const
{
  return output_cloud_;
}

shared_ptr<VectorXf> NaiveTrackAccumulator::getIntensity() const
{
  return output_intensity_;
}
  
string NaiveTrackAccumulator::_getName() const
{
  ostringstream oss;
  oss << "NaiveTrackAccumulator";
  return oss.str();
}
  
void NaiveTrackAccumulator::_flush()
{
  output_cloud_.reset();
  output_intensity_.reset();
}

void NaiveTrackAccumulator::_compute()
{
  assert(interface_->track_);
  Track& track = *interface_->track_;
  
  // -- Get the total number of points.
  int num_points = 0;
  for(size_t i = 0; i < track.frames_.size(); ++i)
    num_points += track.frames_[i]->cloud_->get_points_size();
    
  // -- Allocate memory for the accumulated point cloud and fill it.      
  if(num_points <= max_points_) {
    output_cloud_ = shared_ptr<MatrixXf>(new MatrixXf(num_points, 3));
    output_intensity_ = shared_ptr<VectorXf>(new VectorXf(num_points));
    size_t idx = 0;
    for(size_t i = 0; i < track.frames_.size(); ++i) {
      sensor_msgs::PointCloud& pc = *track.frames_[i]->cloud_;
      for(size_t j = 0; j < pc.get_points_size(); ++j, ++idx) {
	output_cloud_->coeffRef(idx, 0) = pc.points[j].x;
	output_cloud_->coeffRef(idx, 1) = pc.points[j].y;
	output_cloud_->coeffRef(idx, 2) = pc.points[j].z;
	output_intensity_->coeffRef(idx) = pc.channels[0].values[j];
      }
    }
    assert(idx == (size_t)num_points);
  }
  else {
    // -- Downsample to max_points_.
    int num_invalid = num_points - max_points_;
    double interval = (double)num_points / (double)num_invalid;
    output_cloud_ = shared_ptr<MatrixXf>(new MatrixXf(max_points_, 3));
    output_intensity_ = shared_ptr<VectorXf>(new VectorXf(max_points_));
    size_t in_idx = 0;
    size_t out_idx = 0;
    double skip_idx = 0.0;
    for(size_t i = 0; i < track.frames_.size(); ++i) {
      sensor_msgs::PointCloud& pc = *track.frames_[i]->cloud_;
      for(size_t j = 0; j < pc.get_points_size(); ++j, ++in_idx) {
	if(in_idx == (size_t)ceil(interval * skip_idx)) {
	  ++skip_idx;
	  continue;
	}
	assert(out_idx < (size_t)max_points_);
	output_cloud_->coeffRef(out_idx, 0) = pc.points[j].x;
	output_cloud_->coeffRef(out_idx, 1) = pc.points[j].y;
	output_cloud_->coeffRef(out_idx, 2) = pc.points[j].z;
	output_intensity_->coeffRef(out_idx) = pc.channels[0].values[j];
	++out_idx;
      }
    }
    if(out_idx != max_points_) {
      cout << "out_idx == " << out_idx << ", max_points_ == " << max_points_ << endl;
    }
    assert(out_idx == (size_t)max_points_);
    assert((int)skip_idx == num_invalid);
  }
}

} // namespace
