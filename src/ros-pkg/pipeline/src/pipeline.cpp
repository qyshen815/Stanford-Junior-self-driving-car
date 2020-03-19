#include <iostream>
#include <pipeline/pipeline.h>
#include <map>

using namespace std;
using namespace Eigen;
using boost::shared_ptr;

namespace pipeline {

  ComputeNode::ComputeNode() :
    debug_(false),
    disabled_(false),
    num_finished_inputs_(0),
    done_computation_(false),
    started_computation_(false),
    on_queue_(false),
    mutex_(pthread_mutex_t()),
    time_msec_(-1)
  {
  }

  ComputeNode::~ComputeNode()
  {
  }
  
  bool ComputeNode::trylock() {
    if(pthread_mutex_trylock(&mutex_) == EBUSY)
      return false;
    else
      return true;
  }
  
  void ComputeNode::lock() {
    pthread_mutex_lock(&mutex_);
  }
  
  void ComputeNode::unlock() {
    pthread_mutex_unlock(&mutex_);
  }
  
  double ComputeNode::getComputationTime() const {
    assert(done_computation_);
    return time_msec_;
  }
  
  void ComputeNode::display() const {
    _display();
  }
  
  void ComputeNode::_display() const {
    cout << "  " << getFullName() << " has no display functionality." << endl;
  }
  
  void ComputeNode::flush() {
    _flush();
    done_computation_ = false;
    started_computation_ = false;
    time_msec_ = -1;
    num_finished_inputs_ = 0;
    assert(!on_queue_);
  }
  
  void ComputeNode::compute() {
    // -- Extra debugging.  Remove this.
//     int val = pthread_mutex_trylock(&mutex_);
//     if(val != 0) {
//       cout << "Starting computation, but node is locked.  trylock return val: " << val << endl;
//       cout << "EBUSY: " << EBUSY << ", EINVAL: " << EINVAL << ", EAGAIN: " << EAGAIN << endl;
//       assert(val == 0);
//     }

    assert(on_queue_);
    on_queue_ = false;

    //assert(!trylock());
    assert(!started_computation_);
    if(done_computation_)
      cerr << getFullName() << " is done computation, but its compute() function is being called!" << endl;
    assert(!done_computation_);
  
    started_computation_ = true;
  
    timeval start, end;
    gettimeofday(&start, NULL);
    _compute();
    gettimeofday(&end, NULL);
    done_computation_ = true;
    time_msec_ = (end.tv_sec - start.tv_sec) * 1000. + (end.tv_usec - start.tv_usec) / 1000.;

    if(debug_) {
      cout << "Displaying " << getFullName() << endl;
      cout << "  Computation took " << time_msec_ << " ms." << endl;
      display();
    }

    // -- Inform child nodes of completion.
    for(size_t i = 0; i < outputs_.size(); ++i) {
      outputs_[i]->lock();
      ++outputs_[i]->num_finished_inputs_;
      outputs_[i]->unlock();
    }
  }

  uint64_t hashDjb2(const char *str) {
    uint64_t hash = 5381;
    int c;

    // See http://www.cse.yorku.ca/~oz/hash.html.
    while((c = *str++))
      hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
  
    return hash;
  }
  
  string ComputeNode::getShortName() const {
    string ancestors = getGenealogy();
    uint64_t hash = hashDjb2(ancestors.c_str());
    ostringstream oss;
    oss << _getName() << "_" << hash;

    if(oss.str().length() >= getFullName().length())
      return getFullName();

    return oss.str();
  }
  

  string ComputeNode::getGenealogy() const {
    if(inputs_.size() == 0)
      return string("");
    
    ostringstream oss;
    oss << "(";
    for(size_t i = 0; i < inputs_.size(); ++i) { 
      oss << inputs_[i]->getFullName();
      if(i < inputs_.size() - 1)
	oss << "&&";
    }
    oss << ")=>";
    return oss.str();
  }

    
  string ComputeNode::getFullName() const {
    ostringstream oss;
    oss << getGenealogy() << _getName();
    return oss.str();
  }

  void ComputeNode::registerInput(shared_ptr<ComputeNode> input) {
    inputs_.push_back(input.get());
    input->outputs_.push_back(this);
  }

  bool ComputeNode::ready() {
    if(disabled_ || on_queue_ || done_computation_ || started_computation_)
      return false;
    
    if((size_t)num_finished_inputs_ == inputs_.size())
      return true;
    else
      return false;
  }

  DescriptorNode::DescriptorNode() :
    ComputeNode(),
    use_descriptor_(true)
  {
  }

  void DescriptorNode::display() const {
    cout << "  " << printDescriptor() << endl;
    _display();
  }
  
  shared_ptr<VectorXf> DescriptorNode::getDescriptor() const {
    assert(doneComputation());
    return _getDescriptor();
  }

  string DescriptorNode::printDescriptor() const {
    assert(doneComputation());
    ostringstream oss;
    oss << getFullName() << " descriptor (length " << getDescriptorLength() << "): ";
    if(!getDescriptor() || getDescriptor()->rows() == 0)
      oss << "Empty descriptor.";
    else
      oss << getDescriptor()->transpose();
    return oss.str();
  }

  bool useDescriptor(DescriptorNode* node)
  {
    return node->use_descriptor_;
  }


  Pipeline::Pipeline(int num_threads, const vector< shared_ptr<ComputeNode> >& nodes) :
    nodes_(nodes),
    threads_(vector<pthread_t>(num_threads)),
    done_computation_(false),
    num_nodes_computing_(0),
    destructing_(false)
  {
    pthread_mutex_init(&mutex_, NULL);
    pthread_cond_init(&queue_cv_, NULL);
    pthread_cond_init(&done_cv_, NULL);
    
    queue_.reserve(nodes.size());
    
    assertCompleteness();
    startThreadPool();
  }

  Pipeline::~Pipeline() {
    lock();
    destructing_ = true;
    pthread_cond_broadcast(&queue_cv_);
    unlock();
    
    for(size_t i = 0; i < threads_.size(); ++i)
      pthread_join(threads_[i], NULL);

    pthread_mutex_destroy(&mutex_);
    pthread_cond_destroy(&queue_cv_);
    pthread_cond_destroy(&done_cv_);
  }

  void Pipeline::setNodes(const vector< shared_ptr<ComputeNode> >& nodes) {
    //assert(!computing());
    nodes_ = nodes;
  }

  bool Pipeline::trylock() {
    if(pthread_mutex_trylock(&mutex_) == EBUSY)
      return false;
    else
      return true;
  }
  
  void Pipeline::lock() {
    pthread_mutex_lock(&mutex_);
  }
  
  void Pipeline::unlock() {
    pthread_mutex_unlock(&mutex_);
  }

  vector<ComputeNode*> Pipeline::getComponent(ComputeNode* node) const
  {
    queue<ComputeNode*> to_check;
    to_check.push(node);

    set<ComputeNode*> component;
    while(!to_check.empty()) {
      ComputeNode* active = to_check.front();
      to_check.pop();
      component.insert(active); //Won't insert duplicates.
      for(size_t i = 0; i < active->outputs_.size(); ++i) {
	if(component.count(active->outputs_[i]) == 0)
	  to_check.push(active->outputs_[i]);
      }
      for(size_t i = 0; i < active->inputs_.size(); ++i) { 
	if(component.count(active->inputs_[i]) == 0)
	  to_check.push(active->inputs_[i]);
      }
    }

    vector<ComputeNode*> vec;
    vec.reserve(component.size());
    set<ComputeNode*>::const_iterator it;
    for(it = component.begin(); it != component.end(); ++it) {
      vec.push_back(*it);
    }
    return vec;
  }
  
  void Pipeline::assertCompleteness() {
    queue<ComputeNode*> to_check;
    for(size_t i = 0; i < nodes_.size(); ++i) {
      if(nodes_[i]->inputs_.empty())
	to_check.push(nodes_[i].get());
    }

    set<ComputeNode*> found;
    while(!to_check.empty()) {
      ComputeNode* active = to_check.front();
      to_check.pop();
      found.insert(active); //Won't insert duplicates.
      for(size_t i = 0; i < active->outputs_.size(); ++i) {
	to_check.push(active->outputs_[i]);
      }
    }

    assert(found.size() == nodes_.size());  
  }
  
  void Pipeline::flush() {
    num_nodes_computing_ = 0;
    for(size_t i = 0; i < nodes_.size(); ++i) { 
      nodes_[i]->flush();
      assert(!nodes_[i]->done_computation_);
    }
    done_computation_ = false;
  }

  string Pipeline::reportTiming() {
    assert(done_computation_);
  
    ostringstream oss;
    map<string, double> times;
    map<string, double> num_computations;
    for(size_t i = 0; i < nodes_.size(); ++i) {
      times[nodes_[i]->getShortName()] += nodes_[i]->getComputationTime();
      ++num_computations[nodes_[i]->getShortName()];
    }

    assert(times.size() == num_computations.size());
    map<string, double>::iterator cit;
    for(cit = num_computations.begin(); cit != num_computations.end(); ++cit) {
      oss << cit->first << " computed " << cit->second << " times, average time was " << times[cit->first] / cit->second << " ms." << endl;
    }

    return oss.str();
  }

  string sanitize(const string& name) {
    string sani = name.substr(0, name.find_first_of("_"));
    return sani;
  }
  
  string Pipeline::getGraphviz() const {
    ostringstream oss;
    oss << "digraph {" << endl;
    oss << endl;

    for(size_t i = 0; i < nodes_.size(); ++i) {
      oss << (uint64_t)nodes_[i].get() << " [label=\"" << sanitize(nodes_[i]->_getName()) << "\"]" << endl;
    }
    oss << endl;

    for(size_t i = 0; i < nodes_.size(); ++i) {
      vector<ComputeNode*>& outputs = nodes_[i]->outputs_;
      for(size_t j = 0; j < outputs.size(); ++j) {
	oss << (uint64_t)nodes_[i].get() << "->" << (uint64_t)outputs[j] << endl;
      }
    }
    oss << endl;
    
    oss << "}" << endl;
    return oss.str();
  }
  
  void Pipeline::writeGraphviz(const string& filename) const {
    ofstream file;
    file.open(filename.c_str());
    assert(file);
    file << getGraphviz() << endl;
    file.close();
  }

  int Pipeline::switchComponent(ComputeNode* node, bool disabled)
  {
    // -- Ensure one of this Pipeline's nodes is the one being disabled.
    assert(node);
    bool valid = false;
    for(size_t i = 0; i < nodes_.size(); ++i) {
      if(node == nodes_[i].get())
	valid = true;
    }
    assert(valid);

    vector<ComputeNode*> component = getComponent(node);
    for(size_t i = 0; i < component.size(); ++i)
      component[i]->disabled_ = disabled;

    return component.size();
  }

  void Pipeline::enableAll()
  {
    for(size_t i = 0; i < nodes_.size(); ++i)
      nodes_[i]->disabled_ = false;
  }
  
  void Pipeline::disableAll()
  {
    for(size_t i = 0; i < nodes_.size(); ++i)
      nodes_[i]->disabled_ = true;
  }
  
  int Pipeline::disableComponent(ComputeNode* node)
  {
    return switchComponent(node, true);
  }

  int Pipeline::enableComponent(ComputeNode* node)
  {
    return switchComponent(node, false);
  }

  void Pipeline::startThreadPool() {
    for(size_t i=0; i<threads_.size(); ++i) {
      timeval start, end;
      gettimeofday(&start, NULL);
      pthread_create(&(threads_[i]), NULL, propagateComputation, (void*)this);
      gettimeofday(&end, NULL);
    }
  }

  void Pipeline::compute() {
    lock();
    assert(!done_computation_);
    assert(queue_.empty());

    // -- Find all ready nodes and put them into the queue.
    for(size_t i = 0; i < nodes_.size(); ++i) {
      nodes_[i]->lock();
      if(nodes_[i]->ready()) {
	nodes_[i]->on_queue_ = true;
	queue_.push_back(nodes_[i].get());
	pthread_cond_signal(&queue_cv_);
      }
      nodes_[i]->unlock();
    }
    assert(!queue_.empty());

    // -- Wait for the worker nodes to complete.
    pthread_cond_wait(&done_cv_, &mutex_);
    
    done_computation_ = true;
    unlock();
  }
  
  void Pipeline::registerCompleted(ComputeNode* node) { 
    for(size_t i = 0; i < node->outputs_.size(); ++i) {
      node->outputs_[i]->lock();
      if(node->outputs_[i]->ready()) {
	// Debugging.  TODO: remove.
	for(size_t j = 0; j < queue_.size(); ++j)
	  assert(queue_[j] != node->outputs_[i]);
	
	node->outputs_[i]->on_queue_ = true;
	queue_.push_back(node->outputs_[i]);
	pthread_cond_signal(&queue_cv_);
      }
      node->outputs_[i]->unlock();
    }
  }

  void Pipeline::run()
  {
    lock();
    while(true) {
      if(destructing_) { 
	unlock();
	break;
      }
            
      if(queue_.empty()) {
	if(num_nodes_computing_ == 0)
	  pthread_cond_signal(&done_cv_);
	
	pthread_cond_wait(&queue_cv_, &mutex_);
      }

      // pthread signal might awaken more than one thread.
      if(!queue_.empty()) {
	ComputeNode* node = queue_.back();
	queue_.pop_back();

	// Debugging. TODO: remove.
	assert(node->on_queue_);
	for(size_t i = 0; i < queue_.size(); ++i)
	  assert(queue_[i] != node);

	++num_nodes_computing_;
	unlock();

	node->lock();
	node->compute();
	node->unlock();

	lock();
	registerCompleted(node);
	--num_nodes_computing_;
      }
    }
  }
  
  void* propagateComputation(void *pipeline)
  {
    Pipeline& pl = *((Pipeline*) pipeline);
    pl.run();
    return NULL;
  }


  
} //namespace
