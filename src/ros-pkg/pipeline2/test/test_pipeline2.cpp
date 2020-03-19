#include <pipeline2/pipeline2.h>
#include <gtest/gtest.h>

#define NUM_THREADS 33

using namespace std;
using namespace pipeline2;
using namespace Eigen;

//! Example of using Outlets to transfer ints.
class ExampleNode : public ComputeNode {
 public:
  int id_;
  Outlet<int> outlet_;
  
  ExampleNode(int id, const vector< Outlet<int>* >& inputs = vector < Outlet<int>* >());
  
 protected:
  vector< Outlet<int>* > inputs_;
  int sum_;

  void _flush();
  void _compute();
  std::string _getName() const;  
};
  
ExampleNode::ExampleNode(int id, const vector< Outlet<int>* >& inputs) :
  ComputeNode(),
  id_(id),
  outlet_(this),
  inputs_(inputs),
  sum_(0)
{
  for(size_t i = 0; i < inputs_.size(); ++i)
    registerInput(inputs_[i]->getNode());

  outlet_.push(0);
}

void ExampleNode::_flush()
{
  sum_ = 0;
  outlet_.push(0);
}

void ExampleNode::_compute() {
  cout << getFullName() << " is computing." << endl;

  ROS_ASSERT(sum_ == 0);
  for(size_t i = 0; i < inputs_.size(); ++i)
    sum_ += inputs_[i]->pull();
  sum_ += id_;
  outlet_.push(sum_);
  cout << "Node " << id_ << " computed " << sum_ << endl;
  usleep(5e5);
  
  cout << getFullName() << " is done." << endl;
}
  
string ExampleNode::_getName() const {
  ostringstream oss;
  oss << "ExampleNode" << id_;
  return oss.str();
}

vector<ComputeNode*> getNodes() {
  vector<ComputeNode*> nodes;
  ExampleNode* parent0(new ExampleNode(0, vector< Outlet<int>* >()));
  ExampleNode* parent1(new ExampleNode(1, vector< Outlet<int>* >()));
  nodes.push_back(parent0);
  nodes.push_back(parent1);
  
  vector< Outlet<int>* > parents;
  parents.push_back(&parent0->outlet_);
  parents.push_back(&parent1->outlet_);

  nodes.push_back(new ExampleNode(2, parents));
  nodes.push_back(new ExampleNode(3, parents));
  nodes.push_back(new ExampleNode(4, parents));
  
  return nodes;
}

ComputeNode* getBranch()
{
  ExampleNode* n0 = new ExampleNode(0);
  ExampleNode* n1 = new ExampleNode(1);

  vector< Outlet<int>* > p0;
  p0.push_back(&n0->outlet_);
  p0.push_back(&n1->outlet_);
  ExampleNode* n2 = new ExampleNode(2, p0);
  ExampleNode* n3 = new ExampleNode(3, p0);

  vector< Outlet<int>* > p1;
  p1.push_back(&n2->outlet_);
  p1.push_back(&n3->outlet_);
  ExampleNode* n4 = new ExampleNode(4, p1);
  
  return n4;
}

TEST(Pipeline2, terminates) {
  cout << "Using " << NUM_THREADS << " threads." << endl;
  Pipeline2 pl(NUM_THREADS);
  pl.addComponent(getBranch());
//   cout << "Using " << sysconf(_SC_NPROCESSORS_ONLN) << " threads." << endl;
//   Pipeline2 pl(sysconf(_SC_NPROCESSORS_ONLN), getNodes());
  pl.compute();
  pl.flush();
  pl.compute();
  EXPECT_TRUE(true);
}

TEST(Pipeline2, changeNumThreads)
{
  Pipeline2 pl(1);
  pl.addComponent(getBranch());
  pl.compute();
  cout << "************************************************************" << endl;
  cout << pl.reportTiming() << endl;
  pl.flush();

  pl.setNumThreads(10);
  pl.compute();
  cout << "************************************************************" << endl;
  cout << pl.reportTiming() << endl;
}

TEST(Pipeline2, GraphViz) {
  Pipeline2 pl(1, getNodes());
  cout << pl.getGraphviz() << endl;
  pl.writeGraphviz("test_graphviz.dot");
}

bool id0(ExampleNode* node) {
  if(node->id_ == 0)
    return true;
  else
    return false;
}

bool disabled(ComputeNode* node) {
  if(node->disabled_)
    return true;
  else
    return false;
}

class ExampleNode2 : public ComputeNode {
 protected:
  void _flush() {}
  void _compute() {}
  std::string _getName() const {return string("foo");}
};

vector<ComputeNode*> getNodes2() {
  vector< ComputeNode* > nodes = getNodes();
  nodes.push_back(new ExampleNode2());
  nodes.back()->disabled_ = true;
  nodes.push_back(new ExampleNode2());
  nodes.back()->disabled_ = true;
  return nodes;
}
	
TEST(Pipeline2, filterNodes) {
  Pipeline2 pl(sysconf(_SC_NPROCESSORS_ONLN), getNodes2());

  vector<ExampleNode2*> examplenode2s = pl.filterNodes<ExampleNode2>();
  EXPECT_TRUE(examplenode2s.size() == 2);
  
  vector<ComputeNode*> all_disabled = pl.filterNodes<ComputeNode>(disabled);
  EXPECT_TRUE(all_disabled.size() == 2);

  vector<ExampleNode*> id0_only = pl.filterNodes<ExampleNode>(id0);
  EXPECT_TRUE(id0_only.size() == 1);
  id0_only[0]->disabled_ = true;

  all_disabled = pl.filterNodes<ComputeNode>(disabled);
  EXPECT_TRUE(all_disabled.size() == 3);
}

TEST(Pipeline2, getShortName) {
  Pipeline2 pl(1, getNodes());
  for(size_t i = 0; i < pl.nodes_.size(); ++i)
    cout << pl.nodes_[i]->getShortName() << endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
