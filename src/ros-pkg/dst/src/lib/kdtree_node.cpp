#include <dst/kdtree_node.h>

using namespace std;

namespace dst
{

  KdTreeNode::KdTreeNode(pipeline2::Outlet<KinectCloud::Ptr>* pcd_otl,
			 double radius) :
    ComputeNode(),
    kdtree_otl_(this),
    pcd_otl_(pcd_otl),
    radius_(radius),
    current_kdtree_(KdTreeType::Ptr((KdTreeType*)NULL)),
    previous_kdtree_(KdTreeType::Ptr((KdTreeType*)NULL)),
    current_pcd_(KinectCloud::Ptr((KinectCloud*)NULL)),
    previous_pcd_(KinectCloud::Ptr((KinectCloud*)NULL))
  {
    registerInput(pcd_otl_->getNode());
  }

  KdTreeNode::~KdTreeNode()
  {
  }

  void KdTreeNode::swap()
  {
    KdTreeType::Ptr tmp = previous_kdtree_;
    previous_kdtree_ = current_kdtree_;
    current_kdtree_ = tmp;

    previous_pcd_ = current_pcd_;
    current_pcd_ = KinectCloud::Ptr((KinectCloud*)NULL);
  }

  void KdTreeNode::_compute()
  {
    swap();
    
    current_pcd_ = pcd_otl_->pull();
    if(!current_kdtree_)
      current_kdtree_ = KdTreeType::Ptr(new KdTreeType(false));
    current_kdtree_->setInputCloud(current_pcd_); // TODO: Check that this doesn't accumulate.

    Output out;
    out.current_kdtree_ = current_kdtree_;
    out.previous_kdtree_ = previous_kdtree_;
    out.current_pcd_ = current_pcd_;
    out.previous_pcd_ = previous_pcd_;
    kdtree_otl_.push(out);
  }

  void KdTreeNode::_display() const
  {
  }

  void KdTreeNode::_flush()
  {
    Output out;
    out.current_kdtree_ = KdTreeType::Ptr((KdTreeType*)NULL);
    out.previous_kdtree_ = KdTreeType::Ptr((KdTreeType*)NULL);
    out.current_pcd_ = KinectCloud::Ptr((KinectCloud*)NULL);
    out.previous_pcd_ = KinectCloud::Ptr((KinectCloud*)NULL);
    kdtree_otl_.push(out);
  }

  void KdTreeNode::_reset()
  {
    current_kdtree_.reset();
    previous_kdtree_.reset();
    current_pcd_.reset();
    previous_pcd_.reset();
  }
    
  std::string KdTreeNode::_getName() const
  {
    std::ostringstream oss;
    oss << "KdTreeNode_radius:" << radius_;
    return oss.str();
  }

} // namespace dst
