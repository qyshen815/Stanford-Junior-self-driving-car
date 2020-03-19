#include <dst/surface_normal_epg.h>

using namespace std;
using namespace Eigen;

namespace dst
{

  SurfaceNormalEPG::SurfaceNormalEPG(pipeline2::Outlet<Normals::Ptr>* normals_otl,
				     pipeline2::Outlet<DepthProjector::Output>* index_otl,
				     pipeline2::Outlet<cv::Mat3b>* image_otl) :
    EdgePotentialGenerator(),
    normals_otl_(normals_otl),
    index_otl_(index_otl),
    image_otl_(image_otl)
  {
    registerInput(normals_otl_->getNode());
    registerInput(image_otl_->getNode());
    registerInput(index_otl_->getNode());
  }

  inline
  double SurfaceNormalEPG::computePotential(int idx0, int idx1, Normals::Ptr normals) const
  {
    double val = 0.0;
    for(int i = 0; i < 3; ++i)
      val += normals->at(idx0).normal[i] * normals->at(idx1).normal[i];

    //cout << val << " " << pow(fabs(val), 60) << endl;
    return pow(fabs(val), 60); // TODO: parameterize.
  }
  
  void SurfaceNormalEPG::fillPotentials(int y, int x, int idx)
  {
    cv::Mat1i index = index_otl_->pull().current_index_;
    Normals::Ptr normals = normals_otl_->pull();

    // TODO: make an abstraction for filling neighbors with bounds checking.
    if(y > 0 && index(y-1, x) != -1) { 
      int idx2 = getIdx(y-1, x, index.cols);
      potentials_.insertBack(idx, idx2) = computePotential(index(y, x), index(y-1, x), normals);
    }
    if(x > 0 && index(y, x-1) != -1) { 
      int idx2 = getIdx(y, x-1, index.cols);
      potentials_.insertBack(idx, idx2) = computePotential(index(y, x), index(y, x-1), normals);
    }
    if(x < index.cols - 1 && index(y, x+1) != -1) { 
      int idx2 = getIdx(y, x+1, index.cols);
      potentials_.insertBack(idx, idx2) = computePotential(index(y, x), index(y, x+1), normals);
    }
    if(y < index.rows - 1 && index(y+1, x) != -1) { 
      int idx2 = getIdx(y+1, x, index.cols);
      potentials_.insertBack(idx, idx2) = computePotential(index(y, x), index(y+1, x), normals);
    }
  }
  
  void SurfaceNormalEPG::_compute()
  {
    cv::Mat1i index = index_otl_->pull().current_index_;
    int num_nodes = index.rows * index.cols;
    //if(potentials_.rows() != num_nodes || potentials_.cols() != num_nodes)
    potentials_ = SparseMatrix<double, RowMajor>(num_nodes, num_nodes); // TODO: Don't reallocate every time.

    for(int y = 0; y < index.rows; ++y) {
      for(int x = 0; x < index.cols; ++x) {
	int idx = getIdx(y, x, index.cols);
	potentials_.startVec(idx);
	if(index(y, x) == -1)
	  continue;
	fillPotentials(y, x, idx);
      }
    }
    potentials_.finalize();
    edge_otl_.push(&potentials_);
  }

  void SurfaceNormalEPG::_display() const
  {
    displayEdges(image_otl_->pull());
  }

  void SurfaceNormalEPG::_flush()
  {
    
  }

  std::string SurfaceNormalEPG::_getName() const
  {
    std::ostringstream oss;
    oss << "SurfaceNormalEPG";
    return oss.str();
  }

} // namespace dst
