
#include "timestamped_object_indexer.h" 

using namespace std;


TimestampedObjectIndexer::TimestampedObjectIndexer(double tol, const std::vector<double>& timestamps, const std::vector<T>& objects) :
  tol_(tol),
  min_(FLT_MAX),
  max_(-FLT_MAX),
  index_(vector< pair<double, T> >(timestamps.size())),
  max_num_objects_in_bin_(0)
{
  assert(timestamps.size() == objects.size());
  assert(tol > 0);
  
  buildIndex(timestamps, objects);
}

void TimestampedObjectIndexer::buildIndex(const std::vector<double>& timestamps, const std::vector<T>& objects)
{
  // -- Get max and min timestamps.
  for(size_t i = 0; i < timestamps.size(); ++i) {
    if(timestamps[i] > max_)
      max_ = timestamps[i];
    if(timestamps[i] < min_)
      min_ = timestamps[i];
  }

  // -- Construct the bins.
  int num_bins = ceil((max_ - min_) / tol);
  bins_ = vector< vector<T> >(num_bins);

  // -- Sort the objects.
  assert(index_.size() == timestamps.size());
  for(size_t i = 0; i < index_.size(); ++i)
    index_[i] = pair<double, T>(timestamps[i], objects[i]);

  sort(index_.begin(), index_.end());
  assert(index_[0].first <= index_[1].first);

  // -- Fill the bins.
  size_t j = 0;
  for(size_t i = 0; i < index_.size(); ++i) {
    double ts = index_[i].first;
    while(ts > tol_ * (double)(j + 1))
      ++j;
    
    bins_[j].push_back(index_[i]);
  }

  // -- Compute max_num_objects_in_bin_.
  for(size_t i = 0; i < bins_.size(); ++i) {
    if(bins_[i].size() > max_num_objects_in_bin_)
      max_num_objects_in_bin_ = bins_[i].size();
  }
}
	 
void TimestampedObjectIndexer::getBinContents(double timestamp, vector<T>* objects) const
{
  objects->reserve(objects->size() + max_num_objects_in_bin_);
  
}

size_t TimestampedObjectIndexer::timestampToBin(double timestamp) const
{
  if(timestamp < min_)
    return 0;
  else 
    return floor((timestamp - min_) / tol_);
}
