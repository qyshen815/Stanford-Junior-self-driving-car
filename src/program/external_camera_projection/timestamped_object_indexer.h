#ifndef TIMESTAMPED_OBJECT_INDEXER_H
#define TIMESTAMPED_OBJECT_INDEXER_H

//#include <vector>
#include <utility>


template<typename T>
class TimestampedObjectIndexer
{
public:
  TimestampedObjectIndexer(double tol, const std::vector<double>& timestamps, const std::vector<T>& objects);
  //! Returns all Ts that are within tol_ of timestamp.
  std::vector<T> query(double timestamp) const;

private:
  //! Bin width.
  double tol_;
  double min_;
  double max_;
  std::vector< std::pair<double, T> > index_;
  std::vector< std::vector< std::pair<double, T> > > bins_;
  size_t max_num_objects_in_bin_;

  void getBinContents(double timestamp, std::vector<T>* objects) const;
  void buildIndex(const std::vector<double>& timestamps, const std::vector<T>& objects);
  size_t timestampToBin(double timestamp) const;
};  

#endif // TIMESTAMPED_OBJECT_INDEXER_H
