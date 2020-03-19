#ifndef NODE_POTENTIAL_GENERATOR_H
#define NODE_POTENTIAL_GENERATOR_H

#include <Eigen/Eigen>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pipeline2/pipeline2.h>
#include <dst/helper_functions.h>

namespace dst
{

  //! Abstract base class for ComputeNodes that compute graph node potentials.
  class NodePotentialGenerator : public pipeline2::ComputeNode
  {
  public:
    pipeline2::Outlet<Eigen::MatrixXd*> source_otl_;
    pipeline2::Outlet<Eigen::MatrixXd*> sink_otl_;
        
    NodePotentialGenerator();
    
  protected:
    Eigen::MatrixXd source_potentials_;
    Eigen::MatrixXd sink_potentials_;

    void displayNodePotentials(const cv::Mat3b background = cv::Mat3b()) const;
    virtual std::string _getName() const = 0;
  };

}

#endif // NODE_POTENTIAL_GENERATOR_H
