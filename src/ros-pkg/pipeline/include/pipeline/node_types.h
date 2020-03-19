#ifndef PIPELINE_NODE_TYPES_H_
#define PIPELINE_NODE_TYPES_H_

#include <pipeline/core.h>


/************************************************************
 * When creating a new compute node, you can use this
 * as a template.
 ************************************************************/
// Don't forget:
//  - Call ComputeNode() in the member initialization list.
//  - Use registerInput() in the constructor to specify this node's inputs.

// class NewComputeNode : public pipeline::ComputeNode {
//  public:
//   // Any data that descendant nodes might want goes here.
//
//   NewComputeNode();
//   ~NewComputeNode();
//
//  protected:
//   // Any data that won't be used outside this class goes here.
//
//   void _compute();
//   //! Optional.
//   void _display() const;
//   void _flush();
//   //! By convention, this is <classname>_<param><val>_<param><val>_...
//   std::string _getName() const;
// };


/************************************************************
 * When creating a new descriptor node, you can use this
 * as a template.
 ************************************************************/

// class NewDescriptorNode : public pipeline::DescriptorNode {
//  public:
//   // Any data that descendant nodes might want goes here.
//
//   NewDescriptorNode();
//   ~NewDescriptorNode();
//   int getDescriptorLength() const;
//
//  protected:
//   //! Doesn't have to be called this.
//   boost::shared_ptr<Eigen::VectorXf> descriptor_;
//   // Any data that won't be used outside this class (or has an accessor function) goes here.
//
//   void _compute();
//   void _flush();
//   std::string _getName() const;
//   //! Optional.
//   void _display() const;
//   //! Returns NULL if the descriptor couldn't be computed for some reason.
//   boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const;
// };

// NewDescriptorNode::NewDescriptorNode() :
//   DescriptorNode()
// {
//   // Call registerInput() on any input nodes.
// }
//
// NewDescriptorNode::~NewDescriptorNode() :
//   DescriptorNode()
// {
//   // Clean up your mess.
// }
//
// int NewDescriptorNode::getDescriptorLength() const {
//
// }
//
// void NewDescriptorNode::_compute() {
//   if(descriptor is not computable for some reason) {
//     assert(descriptor_.get() == NULL);
//     return;
//   }
//   
//   // -- Compute the descriptor.
//   descriptor_ = shared_ptr<VectorXf>(new VectorXf(getDescriptorLength()));
//
//
// }
//
// void NewDescriptorNode::_flush() {
//   descriptor_.reset();
//
//   // -- Clear out everything else.
// }
//
// string NewDescriptorNode::_getName() const {
//   ostringstream oss;
//   oss << "NewDescriptorNode_param:" << param_; // Make this string unique for any settings of the parameters.
//   return oss.str();
// }
//
// void NewDescriptorNode::_display() const {
//
// }
//
// shared_ptr<VectorXf> NewDescriptorNode::_getDescriptor() const {
//   return descriptor_;
// }



namespace pipeline {

  //! Defines a common descriptor interface based on the Eigen::VectorXf.
  class DescriptorNode : public ComputeNode {
  public:
    DescriptorNode();
    virtual int getDescriptorLength() const = 0;
    //! NULL if no descriptor could be computed.
    boost::shared_ptr<Eigen::VectorXf> getDescriptor() const;
    std::string printDescriptor() const;
    //! Convenience flag for filtering out descriptor nodes that you don't want to use.
    //! This doesn't actually do anything; use in conjunction with filterNodes.
    bool use_descriptor_;
    
  private:
    void display() const;
    virtual boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const = 0;
  };

  //! Convenience function for use with filterNodes.
  bool useDescriptor(DescriptorNode* node);
  
} // namespace pipeline

#endif // PIPELINE_NODE_TYPES_H_
