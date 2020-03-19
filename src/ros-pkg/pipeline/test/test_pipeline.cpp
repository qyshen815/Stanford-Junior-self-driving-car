#include <pipeline/pipeline.h>
#include <gtest/gtest.h>

#define NUM_THREADS 33

using namespace std;
using namespace pipeline;
using namespace Eigen;
using boost::shared_ptr;


class ExampleNode : public ComputeNode {
 public:
  int id_;
  ExampleNode(int id, const vector< shared_ptr<ComputeNode> >& inputs);
  
 protected:
  void _flush() {};
  void _compute();
  std::string _getName() const;  
};
  
void ExampleNode::_compute() {
  cout << getFullName() << " is computing." << endl;
  usleep(5e5);
  cout << getFullName() << " is done." << endl;
}

ExampleNode::ExampleNode(int id, const vector< shared_ptr<ComputeNode> >& inputs) :
  ComputeNode(),
  id_(id)
{
  for(size_t i = 0; i < inputs.size(); ++i) {
    registerInput(inputs[i]);
  }
}
  
string ExampleNode::_getName() const {
  ostringstream oss;
  oss << "ExampleNode" << id_;
  return oss.str();
}

vector< shared_ptr<ComputeNode> > getNodes() {
  vector< shared_ptr<ComputeNode> > nodes;
  shared_ptr<ComputeNode> parent0(new ExampleNode(0, vector< shared_ptr<ComputeNode> >()));
  shared_ptr<ComputeNode> parent1(new ExampleNode(1, vector< shared_ptr<ComputeNode> >()));
  nodes.push_back(parent0);
  nodes.push_back(parent1);
  
  vector< shared_ptr<ComputeNode> > parents;
  parents.push_back(parent0);
  parents.push_back(parent1);

  nodes.push_back(shared_ptr<ComputeNode>(new ExampleNode(2, parents)));
  nodes.push_back(shared_ptr<ComputeNode>(new ExampleNode(3, parents)));
  nodes.push_back(shared_ptr<ComputeNode>(new ExampleNode(4, parents)));
    
  return nodes;
}

TEST(Pipeline, terminates) {
  cout << "Using " << NUM_THREADS << " threads." << endl;
  Pipeline pl(NUM_THREADS, getNodes());
//   cout << "Using " << sysconf(_SC_NPROCESSORS_ONLN) << " threads." << endl;
//   Pipeline pl(sysconf(_SC_NPROCESSORS_ONLN), getNodes());
  pl.compute();
  EXPECT_TRUE(true);
}

TEST(Pipeline, GraphViz) {
  Pipeline pl(1, getNodes());
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

vector< shared_ptr<ComputeNode> > getNodes2() {
  vector< shared_ptr<ComputeNode> > nodes = getNodes();
  nodes.push_back(shared_ptr<ComputeNode>(new ExampleNode2()));
  nodes.back()->disabled_ = true;
  nodes.push_back(shared_ptr<ComputeNode>(new ExampleNode2()));
  nodes.back()->disabled_ = true;
  return nodes;
}
	
TEST(Pipeline, filterNodes) {
  Pipeline pl(sysconf(_SC_NPROCESSORS_ONLN), getNodes2());

  vector< shared_ptr<ExampleNode2> > examplenode2s = pl.filterNodes<ExampleNode2>();
  EXPECT_TRUE(examplenode2s.size() == 2);
  
  vector< shared_ptr<ComputeNode> > all_disabled = pl.filterNodes<ComputeNode>(disabled);
  EXPECT_TRUE(all_disabled.size() == 2);

  vector< shared_ptr<ExampleNode> > id0_only = pl.filterNodes<ExampleNode>(id0);
  EXPECT_TRUE(id0_only.size() == 1);
  id0_only[0]->disabled_ = true;

  all_disabled = pl.filterNodes<ComputeNode>(disabled);
  EXPECT_TRUE(all_disabled.size() == 3);
}

TEST(Pipeline, getShortName) {
  Pipeline pl(1, getNodes());
  for(size_t i = 0; i < pl.nodes_.size(); ++i)
    cout << pl.nodes_[i]->getShortName() << endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
