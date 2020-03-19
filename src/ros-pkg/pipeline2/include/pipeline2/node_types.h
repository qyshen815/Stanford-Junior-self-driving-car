#ifndef PIPELINE2_NODE_TYPES_H_
#define PIPELINE2_NODE_TYPES_H_

#include <pipeline2/pipeline2.h>


/************************************************************
 * When creating a new compute node, you can use this
 * as a template.
 ************************************************************/
// Don't forget:
//  - Call ComputeNode() in the member initialization list.
//  - Use registerInput() in the constructor to specify this node's inputs.

// class NewComputeNode : public pipeline2::ComputeNode {
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

#endif // PIPELINE2_NODE_TYPES_H_
