#ifndef PIPELINE2_H_
#define PIPELINE2_H_

#include <locale>
#include <list>
#include <sys/time.h>
#include <set>
#include <queue>
#include <stdint.h>
#include <fstream>
#include <pthread.h>
#include <errno.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Eigen>
#include <ros/console.h>
#include <ros/assert.h>
#include <bag_of_tricks/high_res_timer.h>

namespace pipeline2 { 
  
  /*****************************************
   * Classes
   ****************************************/
  
  //! Abstract base class that represents a node in the computation graph.  All nodes inherit from this class, at least indirectly.
  //! In general, _foo() is a member function that is wrapped by foo().
  class ComputeNode {  
  public:
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
    int getNumTimesComputed() const;
    std::string getRunName(int width = 4) const;
    
  protected:
    void registerInput(ComputeNode* input);
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
    int num_times_computed_;
    
    //! Function called after node computes if debug_ == true.
    //! Virtual for overloading by ComputeNode subclasses that are abstract, e.g. DescriptorNode.
    virtual void display() const; 
    //! Performs computation, given data from nodes in inputs_.
    virtual void _compute() = 0;
    //! Clears cached data to prepare for next run.
    virtual void _flush() = 0;
    //! Hard reset; clears everything, including data not cleared by _flush().
    virtual void _reset();
    //! Returns a name that is unique for any parameter settings of this node.
    virtual std::string _getName() const = 0;

    void flush();
    void reset();
    void compute();
    std::string getGenealogy() const;
    bool ready();
    bool trylock();
    void lock();
    void unlock();
  
    friend class Pipeline2;
    friend void* propagateComputation(void *pipeline2);
    friend std::vector<ComputeNode*> getComponent(ComputeNode* node);
  };
  
  //! Class that represents the entire computation graph and manages its execution.
  class Pipeline2 {
  public:
    std::vector<ComputeNode*> nodes_;
    
    Pipeline2(int num_threads = 1, const std::vector<ComputeNode*>& nodes = std::vector<ComputeNode*>());
    ~Pipeline2();

    //! Return all nodes of type T for which the given the test function evaluates to true.
    template<typename T> std::vector<T*> filterNodes(bool (*test)(T* node) = NULL) const;

    //! Adds all nodes that are connected to node.
    void addComponent(ComputeNode* node);
    void setNodes(const std::vector<ComputeNode*>& nodes);
    //! Sets the debug_ flag on all nodes.
    //! If debug is being turned on, it kills the thread pool and restarts it with only one thread.
    //! If debug is being turned off, it kills the thread pool and restarts it with num_threads_.
    void setDebug(bool debug);
    void setNumThreads(int num_threads);
    bool getDebug() const;
    
    //! Clears all cached data in the pipeline2, i.e. calls flush on all nodes.
    void flush();
    //! Calls _reset() on all nodes.
    void reset();
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
    //! Makes sure that all nodes in the pipeline2 are actually in nodes_.
    //! This prevents the possibility of a very nasty bug.
    void assertCompleteness();
  
  private:
    std::vector<pthread_t> threads_;
    //! Not necessarily equal to threads_.size(); see setDebug().
    int num_threads_;
    bool debug_;
    pthread_mutex_t mutex_;
    pthread_cond_t queue_cv_;
    pthread_cond_t done_cv_;

    std::vector<ComputeNode*> queue_;
    bool done_computation_;
    size_t num_nodes_computing_;
    bool destructing_;
    //! The amount of time it took from calling compute() to being done.
    double time_msec_;

    //! No copy constructing of Pipeline2s.
    Pipeline2(const Pipeline2& other);
    //! No assigment of Pipeline2s.
    Pipeline2& operator=(const Pipeline2& p);
    ComputeNode* getReadyNode();
    void registerCompleted(ComputeNode* node);
    bool trylock();
    void lock();
    void unlock();
    //! Wait on the queue condition variable.
    void wait();
    //! Signal the queue condition variable.
    void signal();
    void spawnThreadPool(int num_threads);
    void killThreadPool();
    bool computing();
    int switchComponent(ComputeNode* node, bool disabled);
    void run();
    
    friend void* propagateComputation(void *pipeline2);
  };

  //! Generic class for passing data out of a pipeline::ComputeNode.
  //! T will be copied a bit so it should generally be something small,
  //! like a pointer, shared_ptr, or POD type.
  //!
  //! Convention: nodes that pass out shared_ptrs will reallocate every time
  //! rather than erase the data from under the pointer.  Note that reallocation
  //! is generally expensive and that you should probably be using fixed storage
  //! whenever possible, and in this case your Outlets should pass out
  //! pointers.
  //!
  //! Finally, you should probably prefer const pointers or const shared_ptrs.
  //! Different threads executing downstream nodes are likely to simultaneously read the data
  //! that these pointers point to, so nothing should be modifying them.
  template<typename T>
  class Outlet
  {
  public:
    //! T is set with the default constructor, so you should probably set it with push() after
    //! construction.
    Outlet(ComputeNode* node);
    //! Only the descendent nodes should pull.
    T pull() const;
    //! Only the owner should push.
    //! node_->flush() should call this with something appropriate, like NULL if T is a pointer.
    void push(T data);
    ComputeNode* getNode() const;
    
  protected:
    ComputeNode* node_;
    T data_;
  };

  template<typename T>
  Outlet<T>::Outlet(ComputeNode* node) :
    node_(node)
  {
  }
  
  template<typename T>
  T Outlet<T>::pull() const
  {
    return data_;
  }

  template<typename T>
  void Outlet<T>::push(T data)
  {
    data_ = data;
  }

  template<typename T>
  ComputeNode* Outlet<T>::getNode() const
  {
    return node_;
  }
  

  /****************************************
   * Helper Functions
   ****************************************/

  std::vector<ComputeNode*> getComponent(ComputeNode* node);
  
  //! Returns nodes of type T that pass a user-specified test.
  template<typename T>
  std::vector<T*>
  filterNodes(const std::vector<ComputeNode*>& nodes, bool (*test)(T* node) = NULL);

  //! Function that Pipeline2 worker threads call.
  void* propagateComputation(void *pipeline2);

 
  /*****************************************
   * Function Templates
   ****************************************/
  
  template<typename T>
  std::vector<T*> Pipeline2::filterNodes(bool (*test)(T* node)) const
  {
    return pipeline2::filterNodes<T>(nodes_, test); // pipeline2:: is required; otherwise g++ thinks that we're calling a member function which doesn't exist.
  }
  
  template<typename T>
  std::vector<T*> filterNodes(const std::vector<ComputeNode*>& nodes, bool (*test)(T* node))
  {
    std::vector<T*> passed;
    passed.reserve(nodes.size());
    for(size_t i = 0; i < nodes.size(); ++i) {
      T* casted = dynamic_cast<T*>(nodes[i]);
      if(!casted)
	continue;
      if(test && !test(casted))
	continue;
    
      passed.push_back(casted);
    }
    
    return passed;
  }

} // namespace pipeline2

#endif // PIPELINE2_H_
