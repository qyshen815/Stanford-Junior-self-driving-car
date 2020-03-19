#include <online_learning/online_learning.h>

using namespace Eigen;
using namespace std;

namespace odontomachus { 

  int sampleCategorical(const VectorXd& prob)
  {
    ROS_ASSERT(fabs(prob.sum() - 1.0) < 1e-6);
    for(int i = 0; i < prob.rows(); ++i)
      ROS_ASSERT(prob(i) >= 0);

    double r = (double)rand() / (double)RAND_MAX;
    double cum = 0;
    for(int i = 0; i < prob.rows(); ++i) {
      cum += prob(i);
      if(cum >= r)
	return i;
    }
    return prob.rows() - 1;
  }
  
  DatasetSplitter::DatasetSplitter(const Eigen::VectorXd& probabilities) :
    probabilities_(probabilities)
  {
  }

  vector<int> DatasetSplitter::makeRandomTrackAssigments(Dataset::ConstPtr data) const 
  {
    vector<int> assignments(data->track_end_flags_.sum());
    for(size_t i = 0; i < assignments.size(); ++i)
      assignments[i] = sampleCategorical(probabilities_);
    return assignments;
  }

  vector<int> DatasetSplitter::makeRollingTrackAssignments(Dataset::ConstPtr data) const
  {
    vector<int> assignments(data->track_end_flags_.sum(), -1);
    for(int i = data->class_map_.minId(); i <= data->class_map_.maxId(); ++i)
      makeRollingTrackAssignments(i, data, &assignments);

    for(size_t i = 0; i < assignments.size(); ++i)
      ROS_ASSERT(assignments[i] >= 0);

    return assignments;
  }

  void DatasetSplitter::makeRollingTrackAssignments(int class_id, Dataset::ConstPtr data,
						    vector<int>* assignments) const
  {
    // -- Get total number of tracks with this label.
    int total = 0;
    for(int i = 0; i < data->track_end_flags_.rows(); ++i)
      if(data->track_end_flags_(i) == 1 && data->labels_(i) == class_id)
	++total;
    ROS_ASSERT(class_id < -1 || total >= probabilities_.rows());
    if(total == 0)
      return;

    // -- Determine number of tracks to give to each partition.
    VectorXi desired(probabilities_.rows()); // desired(i) is the number of tracks to put in partition i.
    for(int i = 0; i < desired.rows(); ++i)
      desired(i) = floor(probabilities_(i) * total);
    int leftover = total - desired.sum();
    ROS_ASSERT(leftover < desired.rows());
    for(int i = 0; i < leftover; ++i)
      ++desired(i);

    ROS_ASSERT(desired.sum() == total);
    for(int i = 0; i < desired.rows(); ++i)
      ROS_ASSERT(desired(i) > 0);

    // -- Assign them.
    size_t pid = 0; // Which partition to assign to currently.
    int num_assigned = 0; // How many tracks have been assigned to pid so far.
    size_t track_idx = 0;
    for(int i = 0; i < data->labels_.rows(); ++i) {
      if(data->track_end_flags_(i) == 1 && data->labels_(i) == class_id) {
	ROS_ASSERT(pid < (size_t)desired.rows());
	assignments->at(track_idx) = pid;
	++num_assigned;
	if(num_assigned == desired(pid)) {
	  num_assigned = 0;
	  ++pid;
	}
      }
      if(data->track_end_flags_(i) == 1)
	++track_idx;
    }
  }
  
  
  void DatasetSplitter::splitTrackwise(Dataset::ConstPtr data)
  {
    partitions_.clear();
    
    // -- Decide which track will go where.
    vector<int> assignments; // assignments[i] is the partition that track i should go in to.
    assignments = makeRollingTrackAssignments(data);

    // -- Set up the new datasets.
    vector<size_t> num_instances(probabilities_.rows());
    size_t s = 0;
    size_t track_idx = 0;
    for(int i = 0; i < data->track_end_flags_.rows(); ++i) {
      ++s;
      if(data->track_end_flags_(i) == 1) {
	num_instances[assignments[track_idx]] += s;
	s = 0;
	++track_idx;
      }
    }

    int total = 0;
    for(size_t i = 0; i < num_instances.size(); ++i)
      total += num_instances[i];
    ROS_ASSERT(total == data->labels_.rows());

    for(size_t i = 0; i < num_instances.size(); ++i)
      ROS_DEBUG_STREAM("Partition " << i << " will receive " << num_instances[i] << " instances.");

    partitions_.resize(num_instances.size());
    for(size_t i = 0; i < num_instances.size(); ++i) {
      if(num_instances[i] == 0) { 
	partitions_[i] = Dataset::Ptr((Dataset*)NULL);
	continue;
      }
      
      Dataset::Ptr part(new Dataset());
      part->track_end_flags_ = VectorXi(num_instances[i]);
      part->class_map_ = data->class_map_;
      part->descriptor_map_ = data->descriptor_map_;
      part->labels_ = VectorXi(num_instances[i]);
      part->descriptors_ = MatrixXf(part->descriptor_map_.size(), num_instances[i]);
      partitions_[i] = part;
    }      
    
    // -- Copy the tracks.
    track_idx = 0;
    vector<size_t> index(partitions_.size(), 0); // index[i] is the instance in partitions_[i] to be filled next.
    for(int i = 0; i < data->track_end_flags_.rows(); ++i) {
      Dataset& part = *partitions_[assignments[track_idx]];
      size_t& pidx = index[assignments[track_idx]];

      part.labels_(pidx) = data->labels_(i);
      part.track_end_flags_(pidx) = data->track_end_flags_(i);
      part.descriptors_.col(pidx) = data->descriptors_.col(i);

      ++pidx;
      if(data->track_end_flags_(i))
	++track_idx;
    }

    // -- Make sure we didn't screw up.
    for(size_t i = 0; i < index.size(); ++i) {
      partitions_[i]->assertConsistency();
      ROS_ASSERT(index[i] == (size_t)partitions_[i]->labels_.rows());
    }

    ROS_DEBUG_STREAM("Original dataset: " << data->status());
    for(size_t i = 0; i < partitions_.size(); ++i)
      ROS_DEBUG_STREAM("Partition " << i << endl << partitions_[i]->status());
  }

  void DatasetSplitter::splitFramewise(Dataset::ConstPtr data)
  {
    partitions_.clear();
    
    // -- Make assignments of data to the partitions.
    vector<int> indices(data->labels_.rows());
    for(size_t i = 0; i < indices.size(); ++i)
      indices[i] = i;
    random_shuffle(indices.begin(), indices.end());

    vector< vector<int> > assignments(probabilities_.rows());
    size_t start = 0;
    size_t end = 0;
    for(size_t i = 0; i < assignments.size(); ++i) {
      end = start + probabilities_(i) * data->labels_.rows();
      ROS_ASSERT(end <= indices.size());
      assignments[i].insert(assignments[i].begin(), indices.begin() + start, indices.begin() + end);
      start = end;
    }
    for(size_t i = end; i < indices.size(); ++i) {
      size_t j = i % assignments.size();
      assignments[j].push_back(indices[i]);
    }
    int total = 0;
    for(size_t i = 0; i < assignments.size(); ++i)
      total += assignments[i].size();
    ROS_ASSERT(total == data->labels_.rows());

    // -- Copy the data.
    partitions_.resize(assignments.size());
    for(size_t i = 0; i < assignments.size(); ++i) {
      Dataset::Ptr part(new Dataset());
      part->track_end_flags_ = VectorXi::Ones(assignments[i].size()); // Track data is erased.
      part->class_map_ = data->class_map_;
      part->descriptor_map_ = data->descriptor_map_;
      part->labels_ = VectorXi(assignments[i].size());
      part->descriptors_ = MatrixXf(part->descriptor_map_.size(), assignments[i].size());
      for(size_t j = 0; j < assignments[i].size(); ++j) {
	part->labels_(j) = data->labels_(assignments[i][j]);
	part->descriptors_.col(j) = data->descriptors_.col(assignments[i][j]);
      }
      partitions_[i] = part;
    }

    ROS_DEBUG_STREAM("Original dataset: " << endl << data->status());
    for(size_t i = 0; i < partitions_.size(); ++i)
      ROS_DEBUG_STREAM("Partition " << i << ": " << endl << partitions_[i]->status());
	    
  }

} // namespace
