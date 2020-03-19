#include <online_learning/online_learning.h>

using namespace Eigen;
using namespace std;

namespace odontomachus
{

  Classification::Classification()
  {
  }
  
  Classification::Classification(const Classification& other) :
    response_(other.response_)
  {
  }

  Classification::Classification(const Eigen::VectorXf& response) :
    response_(response)
  {
  }

  void Classification::serialize(std::ostream& out) const
  {
    out << "Classification" << endl;
    out << response_.transpose() << endl;
    out << "classification_id: " << getClassId() << endl;
  }

  void Classification::deserialize(std::istream& in)
  {
    ROS_FATAL("No.");
  }
  
  int Classification::getClassId() const
  {
    int id;
    float max = response_.maxCoeff(&id);
    if(max <= 0)
      id = -1;
    return id;
  }

  float Classification::getConfidence() const
  {
    return fabs(response_.maxCoeff());
  }

} // namespace
