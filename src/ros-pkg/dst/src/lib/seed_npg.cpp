#include <dst/seed_npg.h>

using namespace std;
using namespace Eigen;

namespace dst
{
  
  SeedNPG::SeedNPG(pipeline2::Outlet<cv::Mat1b>* seed_otl) :
    NodePotentialGenerator(),
    seed_otl_(seed_otl)
  {
    registerInput(seed_otl_->getNode());
  }
  
  void SeedNPG::_compute()
  {
    cv::Mat1b seed = seed_otl_->pull();
    
    if(seed.rows != source_potentials_.rows() ||
       seed.cols != source_potentials_.cols()) {
      source_potentials_ = MatrixXd::Zero(seed.rows, seed.cols);
      sink_potentials_ = MatrixXd::Zero(seed.rows, seed.cols);
    }
    
    // -- Assign potentials.
    for(int y = 0; y < seed.rows; ++y) {
      for(int x = 0; x < seed.cols; ++x) {
	if(seed(y, x) == 255) { 
	  source_potentials_(y, x) = 1.0;
	  sink_potentials_(y, x) = 0.0;
	}
	else if(seed(y, x) == 0) {
	  source_potentials_(y, x) = 0.0;
	  sink_potentials_(y, x) = 1.0;
	}
	else {
	  source_potentials_(y, x) = 0.0;
	  sink_potentials_(y, x) = 0.0;
	}
      }
    }

    source_otl_.push(&source_potentials_);
    sink_otl_.push(&sink_potentials_);
  }
  
  void SeedNPG::_display() const
  {
    displayNodePotentials();
  }

  void SeedNPG::_flush()
  {
    source_potentials_.setZero();
    sink_potentials_.setZero();
    
    source_otl_.push(NULL);
    sink_otl_.push(NULL);
  }

  std::string SeedNPG::_getName() const
  {
    return "SeedNPG";
  }

}
