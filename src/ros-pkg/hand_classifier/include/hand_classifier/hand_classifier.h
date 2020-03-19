#ifndef HAND_CLASSIFIER_H
#define HAND_CLASSIFIER_H

#include <pipeline/pipeline.h>
#include <cluster_descriptors/cluster_descriptors.h>
#include <multibooster/multibooster.h>
#include <image_descriptor_nodes/image_descriptor_nodes.h>

class HandClassifierPipeline
{
 public:
  bool debug_;

  HandClassifierPipeline(MultiBooster* booster, bool debug);
  Object* computeMultiBoosterObject(IplImage* img);
  Eigen::VectorXf classify(IplImage* img);
  std::vector<std::string> getDescriptorNames() const;
  NameMapping getClassMap() const;
  int getBytesPerTrainingExample() const;
  void writeGraphviz(const std::string& path) const;
  
 private:
  pipeline::Pipeline pipeline_;
  boost::shared_ptr<ImageNode> image_node_;
  boost::shared_ptr<MultiBoosterObjectConstructor> constructor_;
  boost::shared_ptr<MultiBoosterNode> classifier_;
  
  void initialize(MultiBooster* booster);
  void initializeFixedSize(MultiBooster* booster);
  void initializeBasicHOG(MultiBooster* booster);
};

std::vector<std::string> getClassNames() {
  std::vector<std::string> names;
  names.push_back("hand");
  return names;
}




#endif // HAND_CLASSIFIER_H
