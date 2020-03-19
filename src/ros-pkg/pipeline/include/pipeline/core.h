#ifndef PIPELINE_CORE_H_
#define PIPELINE_CORE_H_

#include <Eigen/Eigen>
#include <pthread.h>
#include <boost/shared_ptr.hpp>
#include <errno.h>
#include <locale>
#include <list>
#include <sys/time.h>
#include <set>
#include <queue>
#include <stdint.h>
#include <fstream>

namespace pipeline { 

  /*****************************************
   * Classes
   ****************************************/
  
  //! Abstract base class that represents a node in the computation graph.  All nodes inherit from this class, at least indirectly.
  //! In general, _foo() is a member function that is wrapped by foo().
  class ComputeNode {  
  public:
    typedef boost::shared_ptr<ComputeNode> Ptr;
    typedef boost::shared_ptr<const ComputeNode> ConstPtr;
    
    //! Whether to pause after computation and display debugging output by running display()
    bool debug_;
    //! To compute, or not to compute?
    bool disabled_;
  
    ComputeNode();
    virtual ~ComputeNode();
    //! Returns a string that is unique to this node for this node's parameters, the set of ancestor nodes, and their parameters.
    std::string getFullName() const;
    //! Returns a string that is unique to this node for the given set of ancestor nodes, yet is still short and informative.
    //! Unlike the STL collate hash, this hash is the same on 32bit and 64bit systems.
    std::string getShortName() const;
    //! Returns computation time in ms.
    double getComputationTime() const;
    
  protected:
    void registerInput(ComputeNode::Ptr input);
    bool doneComputation() const {return done_computation_;}
  
    //! Display function, called on compute() when debug_ == true.
    virtual void _display() const;

  private:
    std::vector<ComputeNode*> inputs_;
    std::vector<ComputeNode*> outputs_;
    int num_finished_inputs_;
    bool done_computation_;
    bool started_computation_;
    bool on_queue_;
    pthread_mutex_t mutex_;
    //! The time it took to compute, in milliseconds.
    double time_msec_;
    

    //! Function called after node computes if debug_ == true.
    //! Virtual for overloading by ComputeNode subclasses that are abstract, e.g. DescriptorNode.
    virtual void display() const; 
    //! Performs computation, given data from nodes in inputs_.
    virtual void _compute() = 0;
    //! Clears all cached data.
    virtual void _flush() = 0;
    //! Returns a name that is unique for any parameter settings of this node.
    virtual std::string _getName() const = 0;

    void flush();
    void compute();
    std::string getGenealogy() const;
    bool ready();
    bool trylock();
    void lock();
    void unlock();
  
    friend class Pipeline;
    friend void* propagateComputation(void *pipeline);
  };
  
  //! Class that represents the entire computation graph and manages its execution.
  class Pipeline {
  public:
    std::vector<ComputeNode::Ptr> nodes_;

    Pipeline(int num_threads = 1, const std::vector<ComputeNode::Ptr>& nodes = std::vector<ComputeNode::Ptr>());
    ~Pipeline();

    //! Return all nodes of type T for which the given the test function evaluates to true.
    template<typename T> std::vector< boost::shared_ptr<T> > filterNodes(bool (*test)(T* node) = NULL) const;
    void setNodes(const std::vector<ComputeNode::Ptr>& nodes);

    //! Clears all cached data in the pipeline, i.e. calls flush on all nodes.
    void flush();
    //! Runs all computation until completion.
    void compute();
    //! Returns a string with timing results for each node name.  Nodes that have identical getShortName()s will be averaged together.
    std::string reportTiming();
    std::string getGraphviz() const;
    void writeGraphviz(const std::string& filename) const;
    //! Disables node and all other nodes connected to it.
    int disableComponent(ComputeNode* node);
    //! Enables node and all other nodes connected to it.
    int enableComponent(ComputeNode* node);
    void enableAll();
    void disableAll();

  protected:
    //! Makes sure that all nodes in the pipeline are actually in nodes_.
    //! This prevents the possibility of a very nasty bug.
    void assertCompleteness();
  
  private:
    std::vector<pthread_t> threads_;
    pthread_mutex_t mutex_;
    pthread_cond_t queue_cv_;
    pthread_cond_t done_cv_;

    std::vector<ComputeNode*> queue_;
    bool done_computation_;
    size_t num_nodes_computing_;
    bool destructing_;

    //! No copy constructing of Pipelines.
    Pipeline(const Pipeline& other);
    //! No assigment of Pipelines.
    Pipeline& operator=(const Pipeline& p);
    ComputeNode* getReadyNode();
    void registerCompleted(ComputeNode* node);
    bool trylock();
    void lock();
    void unlock();
    //! Wait on the queue condition variable.
    void wait();
    //! Signal the queue condition variable.
    void signal();
    void startThreadPool();
    bool computing();
    std::vector<ComputeNode*> getComponent(ComputeNode* node) const;
    int switchComponent(ComputeNode* node, bool disabled);
    void run();
    
    friend void* propagateComputation(void *pipeline);
  };

  /****************************************
   * Helper Functions
   ****************************************/

  //! Returns nodes of type T that pass a user-specified test.
  template<typename T> std::vector< boost::shared_ptr<T> >
    filterNodes(const std::vector<ComputeNode::Ptr>& nodes, bool (*test)(T* node) = NULL);

  //! Function that Pipeline worker threads call.
  void* propagateComputation(void *pipeline);

 
  /*****************************************
   * Function Templates
   ****************************************/

  template<typename T>
    std::vector< boost::shared_ptr<T> > Pipeline::filterNodes(bool (*test)(T* node)) const
  {
    return pipeline::filterNodes<T>(nodes_, test); // pipeline:: is required; otherwise g++ thinks that we're calling a member function which doesn't exist.
  }

  template<typename T>
    std::vector< boost::shared_ptr<T> > filterNodes(const std::vector<ComputeNode::Ptr>& nodes, bool (*test)(T* node))
  {
    std::vector< boost::shared_ptr<T> > passed;
    passed.reserve(nodes.size());
    for(size_t i = 0; i < nodes.size(); ++i) {
      boost::shared_ptr<T> casted = boost::dynamic_pointer_cast<T, ComputeNode>(nodes[i]);
      if(!casted)
	continue;
      if(test && !test(casted.get()))
	continue;
    
      passed.push_back(casted);
    }
    
    return passed;
  }

} // namespace pipeline

#endif // PIPELINE_CORE_H_
