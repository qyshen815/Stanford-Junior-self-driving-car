#ifndef ONLINE_LEARNING_H
#define ONLINE_LEARNING_H

#include <float.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <Eigen/Eigen>
#include <ros/console.h>
#include <ros/assert.h>
#include <iomanip>

#include <eigen_extensions/eigen_extensions.h>
#include <serializable/serializable.h>
#include <name_mapping2/name_mapping2.h>


namespace odontomachus
{
  class Classification : public Serializable
  {
  public:
    typedef boost::shared_ptr<Classification> Ptr;
    typedef boost::shared_ptr<const Classification> ConstPtr;

    Eigen::VectorXf response_;

    Classification();
    Classification(const Classification& other);
    Classification(const Eigen::VectorXf& response);

    int getClassId() const;
    //! If all responses are negative, then return |min|.
    float getConfidence() const;
    
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
  };
  
  class Dataset : public NameMappable, public Serializable
  {
  public:
    typedef boost::shared_ptr<Dataset> Ptr;
    typedef boost::shared_ptr<const Dataset> ConstPtr;

    //! descriptors_(i, j) is the ith descriptor of the jth instance.
    Eigen::MatrixXf descriptors_;
    Eigen::VectorXi labels_;
    //! track_end_flags_[i] == 1 if frame i ends a track; 0 otherwise.
    Eigen::VectorXi track_end_flags_;
    NameMapping2 class_map_;
    NameMapping2 descriptor_map_;

    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
    std::string status() const;
    Dataset& operator+=(const Dataset& other);
    int getNumDescriptors() const;
    //! id 0 is class_map_, id 1 is descriptor_map_.
    void applyNameMapping(const NameTranslator2& translator,
			  const NameMapping2& new_mapping,
			  int id = 0);
    //! id 0 is class_map_, id 1 is descriptor_map_.
    void applyNameMapping(const NameMapping2& new_mapping, int id);
    //! Checks that all fields make sense together.
    void assertConsistency() const;
    
  private:
    void applyClassMap(const NameTranslator2& translator,
		       const NameMapping2& new_mapping);
    void applyDescriptorMap(const NameTranslator2& translator,
			    const NameMapping2& new_mapping);
    
  };

  /* For dividing a dataset into several different components.
   */
  class DatasetSplitter
  {
  public:
    Eigen::VectorXd probabilities_;
    std::vector<Dataset::Ptr> partitions_;
    
    DatasetSplitter(const Eigen::VectorXd& probabilities);
    void splitTrackwise(Dataset::ConstPtr data);
    void splitFramewise(Dataset::ConstPtr data);

  private:
    std::vector<int> makeRollingTrackAssignments(Dataset::ConstPtr data) const;
    std::vector<int> makeRandomTrackAssigments(Dataset::ConstPtr data) const;
    void makeRollingTrackAssignments(int class_id, Dataset::ConstPtr data,
				     std::vector<int>* assignments) const;
	
  };

  /* For chopping out unwanted projections.
   * This should probably not be a class.
   */
  class DatasetSlicer
  {
  public:
    void slice(Dataset::Ptr data, int num_projections) const;
    
  private:
    NameMapping2 sliceDescriptorMap(const NameMapping2& orig, int np) const;
  };
  
  class Params : public Serializable, public std::map<std::string, double>
  {
  public:
    void serialize(std::ostream& out) const;
    void deserialize(std::istream& in);
  };
    
  NameMapping2 getDefaultClassMap();
  NameMapping2 getStubDescriptorMap(int num_descriptors);
}

#endif // ONLINE_LEARNING_H
