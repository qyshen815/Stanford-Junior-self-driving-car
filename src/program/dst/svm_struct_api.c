/***********************************************************************/
/*                                                                     */
/*   svm_struct_api.c                                                  */
/*                                                                     */
/*   Definition of API for attaching implementing SVM learning of      */
/*   structures (e.g. parsing, multi-label classification, HMM)        */ 
/*                                                                     */
/*   Author: Thorsten Joachims                                         */
/*   Date: 03.07.04                                                    */
/*                                                                     */
/*   Copyright (c) 2004  Thorsten Joachims - All rights reserved       */
/*                                                                     */
/*   This software is available for non-commercial use only. It must   */
/*   not be modified and distributed without prior permission of the   */
/*   author. The author is not responsible for implications from the   */
/*   use of this software.                                             */
/*                                                                     */
/***********************************************************************/

#include <stdio.h>
#include <string.h>
#include "svm_struct/svm_struct_common.h"
#include "svm_struct_api.h"

#define NUM_THREADS (getenv("NUM_THREADS") ? atoi(getenv("NUM_THREADS")) : 1)

using namespace std;
namespace bfs = boost::filesystem;
using namespace Eigen;
using namespace dst;

void        svm_struct_learn_api_init(int argc, char* argv[])
{
  /* Called in learning part before anything else is done to allow
     any initializations that might be necessary. */
}

void        svm_struct_learn_api_exit()
{
  /* Called in learning part at the very end to allow any clean-up
     that might be necessary. */
}

void        svm_struct_classify_api_init(int argc, char* argv[])
{
  /* Called in prediction part before anything else is done to allow
     any initializations that might be necessary. */
}

void        svm_struct_classify_api_exit()
{
  /* Called in prediction part at the very end to allow any clean-up
     that might be necessary. */
}

SAMPLE      read_struct_examples(char *file, STRUCT_LEARN_PARM *sparm)
{
  /* Reads struct examples and returns them in sample. The number of
     examples must be written into sample.n */
  SAMPLE   sample;  /* sample */
  EXAMPLE  *examples;
  long     n;       /* number of examples */


  std::vector<KinectSequence::Ptr> sequences;
  bfs::directory_iterator end_itr; // default construction yields past-the-end
  n = 0;
  cout << endl;
  for(bfs::directory_iterator itr(file); itr != end_itr; ++itr) {
    string path = itr->path().string();
    if(!bfs::is_directory(path))
      continue;

    KinectSequence::Ptr seq(new KinectSequence());
    seq->load(path);
    cout << "Loaded sequence " << path << " with  " << seq->images_.size() << " frames." << endl;
    n += seq->segmentations_.size();
    sequences.push_back(seq);
  }
    
  // -- Cache potentials.
  vector<PotentialsCache::Ptr> caches;
  for(size_t i = 0; i < sequences.size(); ++i) {
    KinectSequence::Ptr seq = sequences[i];
    SegmentationPipeline sp(NUM_THREADS, seq->camera_info_);
    PotentialsCache::Ptr cache = sp.cacheUnweightedPotentialsWithOracle(seq);
    caches.push_back(cache);
    
    /* for(size_t j = 0; j < cache->framecaches_.size(); ++j) { */
    /*   FramePotentialsCache::Ptr fc = cache->framecaches_[j]; */
    /*   cout << "frame " << j << endl; */
    /*   for(size_t k = 0; k < fc->edge_potentials_.size(); ++k) */
    /* 	cout << "edge potentials " << k << " sum:" << fc->edge_potentials_[k].sum() << endl; */
    /*   for(size_t k = 0; k < fc->source_potentials_.size(); ++k) */
    /* 	cout << "source potentials " << k << " sum:" << fc->source_potentials_[k].sum() << endl; */
    /*   for(size_t k = 0; k < fc->sink_potentials_.size(); ++k) */
    /* 	cout << "sink potentials " << k << " sum:" << fc->sink_potentials_[k].sum() << endl; */
    /* } */
  }

  // -- Construct the objects since malloc doesn't do this for us.
  examples = (EXAMPLE *)my_malloc(sizeof(EXAMPLE)*n);
  //examples = new EXAMPLE[n]; // This is probably bad.  Presumably he uses free().
  for(int i = 0; i < n; ++i) {
    new(&examples[i].y.labels_) cv::Mat1b;
    new(&examples[i].x.seq_) KinectSequence::Ptr;
    new(&examples[i].x.cache_) FramePotentialsCache::Ptr;
  }

  // -- Fill individual examples.
  int idx = 0;
  for(size_t i = 0; i < sequences.size(); ++i) {
    KinectSequence::Ptr seq = sequences[i];
    PotentialsCache::Ptr cache = caches[i];
    for(size_t j = 0; j < seq->segmentations_.size(); ++j, ++idx) { 
      examples[idx].x.seq_ = seq;
      examples[idx].x.cache_ = cache->framecaches_[j];
      examples[idx].x.id_ = idx;
        
      examples[idx].y.labels_ = seq->segmentations_[j].clone();
      examples[idx].y.id_ = idx;

      /* cout << "idx: " << idx << endl; */
      /* cout << "cache pointer: " << examples[idx].x.cache_.get() << endl; */
      /* cv::imshow("Ground Truth", examples[idx].y.labels_); */
      /* cv::imshow("di0", examples[idx].x.cache_->depth_index_); */
      /* cv::imshow("di1", cache->framecaches_[j]->depth_index_); */
      /* cv::waitKey(); */

      cv::imshow("aoeu", cache->framecaches_[j]->depth_index_);
      cv::waitKey(100);
    }
  }

  cout << "Done caching oracle potentials." << endl;
  sample.n = n;
  sample.examples = examples;
  return(sample);
}

void        init_struct_model(SAMPLE sample, STRUCTMODEL *sm, 
			      STRUCT_LEARN_PARM *sparm, LEARN_PARM *lparm, 
			      KERNEL_PARM *kparm)
{
  /* Initialize structmodel sm. The weight vector w does not need to be
     initialized, but you need to provide the maximum size of the
     feature space in sizePsi. This is the maximum number of different
     weights that can be learned. Later, the weight vector w will
     contain the learned weights for the model. */

  SegmentationPipeline sp(NUM_THREADS, opencv_candidate::Camera());
  VectorXd weights = sp.getWeights();
  sm->sizePsi = weights.rows();
  cout << "init_struct_model: sm->sizePsi is " << sm->sizePsi << endl;
  /* sm->w = (double*)my_malloc(sizeof(double)*sm->sizePsi); // TODO: This is probably a memory leak. */
  /* for(int i = 0; i < weights.rows(); ++i) */
  /*   sm->w[i] = weights(i);  */
}

CONSTSET    init_struct_constraints(SAMPLE sample, STRUCTMODEL *sm, 
				    STRUCT_LEARN_PARM *sparm)
{
  /* Initializes the optimization problem. Typically, you do not need
     to change this function, since you want to start with an empty
     set of constraints. However, if for example you have constraints
     that certain weights need to be positive, you might put that in
     here. The constraints are represented as lhs[i]*w >= rhs[i]. lhs
     is an array of feature vectors, rhs is an array of doubles. m is
     the number of constraints. The function returns the initial
     set of constraints. */



  CONSTSET c;
  long     sizePsi=sm->sizePsi;
  long     i;
  WORD     words[2];

  // TODO: Switch to only enforcing nonnegativity of edge weights.
  if(0) { /* normal case: start with empty set of constraints */
    c.lhs=NULL;
    c.rhs=NULL;
    c.m=0;
  }
  else { /* add constraints so that all learned weights are
            positive. WARNING: Currently, they are positive only up to
            precision epsilon set by -e. */
    /* cout << "sm->sizePsi: " << sm->sizePsi << endl; */
    /* cout << "sizePsi: " << sizePsi << endl; */
    /* cout << "Allocating " << sizeof(DOC *)*sizePsi << endl; */
    /* cout << "Allocating " << sizeof(double)*sizePsi << endl; */
    c.m = sizePsi;
    c.lhs=(DOC**)my_malloc(sizeof(DOC *)*sizePsi);
    c.rhs=(double*)my_malloc(sizeof(double)*sizePsi);
    for(i=0; i<sizePsi; i++) {
      words[0].wnum=i+1;
      words[0].weight=1.0;
      words[1].wnum=0;
      /* the following slackid is a hack. we will run into problems,
         if we have move than 1000000 slack sets (ie examples) */
      c.lhs[i]=create_example(i,0,1000000+i,1,create_svector(words,NULL,1.0));
      c.rhs[i]=0.0;
    }
  }

  return(c);


  /* CONSTSET c; */
  /* //long     sizePsi=sm->sizePsi; */
  /* long     i; */
  /* WORD     words[2]; */

  /* // -- Put non-negativity constraints on the edge weights. */
  /* SegmentationPipeline sp(NUM_THREADS, opencv_candidate::Camera()); */
  /* int num_edge_weights = sp.getEdgeWeights().rows(); */

  /* c.m = num_edge_weights; */
  /* c.lhs = (DOC**) my_malloc(sizeof(DOC *) * num_edge_weights); */
  /* c.rhs = (double*) my_malloc(sizeof(double) * num_edge_weights); */
  /* for(i = 0; i < num_edge_weights; i++) { */
  /*   words[0].wnum = i + 1; */
  /*   words[0].weight = 1.0; */
  /*   words[1].wnum = 0; // End-of-sparse-vector flag. */
  /*   /\* the following slackid is a hack. we will run into problems, */
  /*      if we have move than 1000000 slack sets (ie examples) *\/ */
  /*   c.lhs[i] = create_example(i, 0, 1000000+i, 1, create_svector(words, "", 1.0)); */
  /*   c.rhs[i] = 0.0; */
  /* } */
  /* return(c); */
}

LABEL       classify_struct_example(PATTERN x, STRUCTMODEL *sm, 
				    STRUCT_LEARN_PARM *sparm)
{
  /* Finds the label yhat for pattern x that scores the highest
     according to the linear evaluation function in sm, especially the
     weights sm.w. The returned label is taken as the prediction of sm
     for the pattern x. The weights correspond to the features defined
     by psi() and range from index 1 to index sm->sizePsi. If the
     function cannot find a label, it shall return an empty label as
     recognized by the function empty_label(y). */
  LABEL y;

  /* insert your code for computing the predicted label y here */
  ROS_ASSERT(0);

  return(y);
}

LABEL       find_most_violated_constraint_slackrescaling(PATTERN x, LABEL y, 
						     STRUCTMODEL *sm, 
						     STRUCT_LEARN_PARM *sparm)
{

  ROS_ASSERT(0);
  
  /* Finds the label ybar for pattern x that that is responsible for
     the most violated constraint for the slack rescaling
     formulation. For linear slack variables, this is that label ybar
     that maximizes

            argmax_{ybar} loss(y,ybar)*(1-psi(x,y)+psi(x,ybar)) 

     Note that ybar may be equal to y (i.e. the max is 0), which is
     different from the algorithms described in
     [Tschantaridis/05]. Note that this argmax has to take into
     account the scoring function in sm, especially the weights sm.w,
     as well as the loss function, and whether linear or quadratic
     slacks are used. The weights in sm.w correspond to the features
     defined by psi() and range from index 1 to index
     sm->sizePsi. Most simple is the case of the zero/one loss
     function. For the zero/one loss, this function should return the
     highest scoring label ybar (which may be equal to the correct
     label y), or the second highest scoring label ybar, if
     Psi(x,ybar)>Psi(x,y)-1. If the function cannot find a label, it
     shall return an empty label as recognized by the function
     empty_label(y). */
  LABEL ybar;

  /* insert your code for computing the label ybar here */

  return(ybar);
}

LABEL       find_most_violated_constraint_marginrescaling(PATTERN x, LABEL y, 
						     STRUCTMODEL *sm, 
						     STRUCT_LEARN_PARM *sparm)
{
  /* Finds the label ybar for pattern x that that is responsible for
     the most violated constraint for the margin rescaling
     formulation. For linear slack variables, this is that label ybar
     that maximizes

            argmax_{ybar} loss(y,ybar)+psi(x,ybar)

     Note that ybar may be equal to y (i.e. the max is 0), which is
     different from the algorithms described in
     [Tschantaridis/05]. Note that this argmax has to take into
     account the scoring function in sm, especially the weights sm.w,
     as well as the loss function, and whether linear or quadratic
     slacks are used. The weights in sm.w correspond to the features
     defined by psi() and range from index 1 to index
     sm->sizePsi. Most simple is the case of the zero/one loss
     function. For the zero/one loss, this function should return the
     highest scoring label ybar (which may be equal to the correct
     label y), or the second highest scoring label ybar, if
     Psi(x,ybar)>Psi(x,y)-1. If the function cannot find a label, it
     shall return an empty label as recognized by the function
     empty_label(y). */
  LABEL ybar;
  ybar.id_ = x.id_;

  ROS_ASSERT(x.id_ == y.id_);
  ROS_ASSERT(sparm->slack_norm != 2);
  ROS_ASSERT(sparm->loss_type == 2); // MARGIN_RESCALING
  cout << "-------------------- Finding most violated..." << endl;
  cout << "ID: " << x.id_ << " " << y.id_ << endl;
  cout << "Kernel type: " << sm->svm_model->kernel_parm.kernel_type << endl;
  cout << "Weights: ";
  double* w = sm->svm_model->lin_weights;
  for(int i = 0; i < sm->sizePsi; ++i)
    cout << w[i] << " ";
  cout << endl;
  cout << "b: " << sm->svm_model->b << endl;

  // -- Set up segmentation using the new weights.
  SegmentationPipeline sp(NUM_THREADS, x.seq_->camera_info_);
  VectorXd weights(sm->sizePsi);
  for(int i = 0; i < weights.rows(); ++i) {
    if(i < sp.getEdgeWeights().rows() && sm->w[i] < 0) {
      ROS_WARN_STREAM("Edge weight is " << sm->w[i] << ".  Clipping to zero.");
      weights(i) = 0;
    }
    else
      weights(i) = sm->w[i];
  }
  cout << "@@ Using weights: " << weights.transpose() << endl;
  sp.setWeights(weights);

  // Save weights.
  static int widx = 0;
  ostringstream oss;
  oss << "weights" << setw(5) << setfill('0') << widx << ".eig";
  eigen_extensions::save(weights, oss.str());
  cout << "@@ Saved weights to " << oss.str() << endl;
  ++widx;
  
  // -- Solve for most violated constraint.
  if(sparm->loss_function == 0) { /* type 0 loss: 0/1 loss */
    ybar.labels_ = sp.findMostViolatedConstraintMarginRescaling(*x.cache_,
								y.labels_, false);
  }
  else { 
    ybar.labels_ = sp.findMostViolatedConstraintMarginRescaling(*x.cache_,
								y.labels_);
  }

  cout << "Loss between ground truth and most violating: " << loss(y, ybar, sparm) << endl;
  
  // TODO: If equal, this should return the second-best labeling.
  /* cv::Mat1b label = y.labels_; */
  /* cv::Mat1b pred = ybar.labels_; */
  /* bool equal = true; */
  /* for(int y = 0; y < label.rows && equal; ++y) */
  /*   for(int x = 0; x < label.cols && equal; ++x) */
  /*     if(label(y, x) != pred(y, x)) */
  /* 	equal = false; */

  /* if(equal) { */
  /*   ROS_INFO_STREAM("find_most_violated_constraint_marginrescaling found a labeling that matches ground truth."); */
  /*   ybar.labels_ = cv::Mat1b(); */
  /* } */
    
  return(ybar);
}

int         empty_label(LABEL y)
{
  /* Returns true, if y is an empty label. An empty label might be
     returned by find_most_violated_constraint_???(x, y, sm) if there
     is no incorrect label that can be found for x, or if it is unable
     to label x at all */
  if(y.labels_.rows == 0)
    return true;
  else
    return false;
}


ostream& operator<<(ostream& out, const SVECTOR& sv)
{
  out << "Sparse vector" << endl;
  out << " kernel_id: " << sv.kernel_id << endl;
  out << " next: " << sv.next << endl;
  out << " factor: " << sv.factor << endl;

  out << " words: " << endl;
  int i = 0;
  while(true) {
    out << "   " << sv.words[i].wnum << ": " << sv.words[i].weight << endl;
    if(sv.words[i].wnum == 0)
      break;
    ++i;
  }

  return out;
}

SVECTOR     *psi(PATTERN pattern, LABEL label, STRUCTMODEL *sm,
		 STRUCT_LEARN_PARM *sparm)
{
  /* Returns a feature vector describing the match between pattern x
     and label y. The feature vector is returned as a list of
     SVECTOR's. Each SVECTOR is in a sparse representation of pairs
     <featurenumber:featurevalue>, where the last pair has
     featurenumber 0 as a terminator. Featurenumbers start with 1 and
     end with sizePsi. Featuresnumbers that are not specified default
     to value 0. As mentioned before, psi() actually returns a list of
     SVECTOR's. Each SVECTOR has a field 'factor' and 'next'. 'next'
     specifies the next element in the list, terminated by a NULL
     pointer. The list can be though of as a linear combination of
     vectors, where each vector is weighted by its 'factor'. This
     linear combination of feature vectors is multiplied with the
     learned (kernelized) weight vector to score label y for pattern
     x. Without kernels, there will be one weight in sm.w for each
     feature. Note that psi has to match
     find_most_violated_constraint_???(x, y, sm) and vice versa. In
     particular, find_most_violated_constraint_???(x, y, sm) finds
     that ybar!=y that maximizes psi(x,ybar,sm)*sm.w (where * is the
     inner vector product) and the appropriate function of the
     loss + margin/slack rescaling method. See that paper for details. */
  SVECTOR *fvec=NULL;

//  ROS_ASSERT(sparm->newconstretrain == 3);
  
  /* insert code for computing the feature vector for x and y here */
  cout << pattern.id_ << " " << label.id_ << endl;
  ROS_ASSERT(pattern.id_ == label.id_);
  ROS_ASSERT(pattern.cache_);
  FramePotentialsCache& framecache = *pattern.cache_;
  ROS_ASSERT(!empty_label(label));
  cv::Mat1b labels = label.labels_;
  
  // -- Get feature values for edge weights.
  WORD words[framecache.getNumWeights() + 1];
  for(int i = 0; i < framecache.getNumEdgeWeights(); i++) {
    words[i].wnum = i + 1;
    double w = 0;

    // Use symmetric edge potentials.
    // This must match segmentation_pipeline.cpp's run() method.
    SparseMatrix<double, Eigen::RowMajor>& epot = framecache.edge_potentials_[i];
    SparseMatrix<double, Eigen::RowMajor> sym = (epot + epot.transpose()) / 2.0;
    for(int j = 0; j < sym.outerSize(); ++j) {
      for(SparseMatrix<double, RowMajor>::InnerIterator it(sym, j); it; ++it) {
	if(it.col() <= it.row())
	  continue;

	int idx1 = it.col();
	int y1 = idx1 / labels.cols;
	int x1 = idx1 - y1 * labels.cols;
	int idx2 = it.row();
	int y2 = idx2 / labels.cols;
	int x2 = idx2 - y2 * labels.cols;

	// Pixels that don't have depth should automatically be ignored.
	if(framecache.depth_index_(y1, x1) == -1 || framecache.depth_index_(y2, x2) == -1)
	  ROS_ASSERT(it.value() == 0);

	if(labels(y1, x1) == labels(y2, x2))
	  w += it.value();
      }
    }

    words[i].weight = w;
  }

  // -- Get feature values for node weights;
  for(int i = framecache.getNumEdgeWeights(); i < framecache.getNumWeights(); ++i) {
    words[i].wnum = i + 1;
    double w = 0;

    MatrixXd& srcpot = framecache.source_potentials_[i - framecache.getNumEdgeWeights()];
    MatrixXd& snkpot = framecache.sink_potentials_[i - framecache.getNumEdgeWeights()];
    for(int y = 0; y < labels.rows; ++y) {
      for(int x = 0; x < labels.cols; ++x) {
	if(framecache.depth_index_(y, x) == -1) {
	  ROS_ASSERT(labels(y, x) == 127);
	  ROS_ASSERT(srcpot(y, x) == 0 && snkpot(y, x) == 0);
	}
	
	if(labels(y, x) == 255)
	  w += srcpot(y, x);
	else if(labels(y, x) == 0)
	  w += snkpot(y, x);
      }
    }
    
    words[i].weight = w;
  }
  words[framecache.getNumWeights()].wnum = 0; // End-of-sparse-vector flag.
  words[framecache.getNumWeights()].weight = 0;
    
  fvec = create_svector(words, "", 1.0);

  cout << "==================== psi vector ====================" << endl;
  cout << *fvec << endl;

  SegmentationPipeline sp(NUM_THREADS, opencv_candidate::Camera());
  VectorXd psi;
  framecache.computeScore(labels, sp.getEdgeWeights(), sp.getNodeWeights(), &psi);
  cout << psi.transpose() << endl;
  
  return(fvec);
}

double      loss(LABEL y, LABEL ybar, STRUCT_LEARN_PARM *sparm)
{
  ROS_ASSERT(ybar.id_ == y.id_);
  
  /* loss for correct label y and predicted label ybar. The loss for
     y==ybar has to be zero. sparm->loss_function is set with the -l option. */
  if(sparm->loss_function == 0) { /* type 0 loss: 0/1 loss */
                                  /* return 0, if y==ybar. return 1 else */

    cv::Mat1b label = y.labels_;
    cv::Mat1b pred = ybar.labels_;
    ROS_ASSERT(label.rows == pred.rows);
    ROS_ASSERT(label.cols == pred.cols);
    
    bool equal = true;
    for(int y = 0; y < label.rows && equal; ++y) {
      for(int x = 0; x < label.cols && equal; ++x) {
	// Ignore pixels without depth.
	if((label(y, x) == 255 && pred(y, x) == 0) ||
	   (label(y, x) == 0 && pred(y, x) == 255))
	  equal = false;
      }
    }
    if(!equal)
      return 1.0;
    else
      return 0.0;
  }
  else {
    /* Put your code for different loss functions here. But then
       find_most_violated_constraint_???(x, y, sm) has to return the
       highest scoring label with the largest loss. */
    
    // -- Hamming loss.
    cv::Mat1b label = y.labels_;
    cv::Mat1b pred = ybar.labels_;
    ROS_ASSERT(label.rows == pred.rows);
    ROS_ASSERT(label.cols == pred.cols);
    
    double loss = 0;
    for(int y = 0; y < label.rows; ++y) {
      for(int x = 0; x < label.cols; ++x) {
	// Ignore pixels without depth.
	if(label(y, x) == 255 && pred(y, x) == 0)
	  ++loss;
	else if(label(y, x) == 0 && pred(y, x) == 255)
	  ++loss;
      }
    }
    return loss; // TODO: Should this be averaged?
  }

  ROS_ASSERT(0);
  return 0;
}

int         finalize_iteration(double ceps, int cached_constraint,
			       SAMPLE sample, STRUCTMODEL *sm,
			       CONSTSET cset, double *alpha, 
			       STRUCT_LEARN_PARM *sparm)
{
  /* This function is called just before the end of each cutting plane iteration. ceps is the amount by which the most violated constraint found in the current iteration was violated. cached_constraint is true if the added constraint was constructed from the cache. If the return value is FALSE, then the algorithm is allowed to terminate. If it is TRUE, the algorithm will keep iterating even if the desired precision sparm->epsilon is already reached. */
  return(0);
}

void        print_struct_learning_stats(SAMPLE sample, STRUCTMODEL *sm,
					CONSTSET cset, double *alpha, 
					STRUCT_LEARN_PARM *sparm)
{
  /* This function is called after training and allows final touches to
     the model sm. But primarly it allows computing and printing any
     kind of statistic (e.g. training error) you might want. */
}

void        print_struct_testing_stats(SAMPLE sample, STRUCTMODEL *sm,
				       STRUCT_LEARN_PARM *sparm, 
				       STRUCT_TEST_STATS *teststats)
{
  /* This function is called after making all test predictions in
     svm_struct_classify and allows computing and printing any kind of
     evaluation (e.g. precision/recall) you might want. You can use
     the function eval_prediction to accumulate the necessary
     statistics for each prediction. */
}

void        eval_prediction(long exnum, EXAMPLE ex, LABEL ypred, 
			    STRUCTMODEL *sm, STRUCT_LEARN_PARM *sparm, 
			    STRUCT_TEST_STATS *teststats)
{
  /* This function allows you to accumlate statistic for how well the
     predicition matches the labeled example. It is called from
     svm_struct_classify. See also the function
     print_struct_testing_stats. */
  if(exnum == 0) { /* this is the first time the function is
		      called. So initialize the teststats */
  }
}

void        write_struct_model(char *file, STRUCTMODEL *sm, 
			       STRUCT_LEARN_PARM *sparm)
{
  /* Writes structural model sm to file file. */

  VectorXd weights(sm->sizePsi);

  SegmentationPipeline sp(NUM_THREADS, opencv_candidate::Camera());
  for(int i = 0; i < weights.rows(); ++i) {
    if(i < sp.getEdgeWeights().rows() && sm->w[i] < 0) {
      ROS_WARN_STREAM("Edge weight is " << sm->w[i] << ".  Clipping to zero.");
      weights(i) = 0;
    }
    else
      weights(i) = sm->w[i];
  }
  //weights.normalize();
  eigen_extensions::save(weights, file);
  
  cout << "****************************************" << endl;
  cout << "Wrote weights to " << file << endl;
  cout << weights.transpose() << endl;
}

STRUCTMODEL read_struct_model(char *file, STRUCT_LEARN_PARM *sparm)
{
  /* Reads structural model sm from file file. This function is used
     only in the prediction module, not in the learning module. */
  ROS_ASSERT(0);
  STRUCTMODEL sm;
  return sm;
}

void        write_label(FILE *fp, LABEL y)
{
  ROS_ASSERT(0);
} 

void        free_pattern(PATTERN x) {
  /* Frees the memory of x. */
}

void        free_label(LABEL y) {
  /* Frees the memory of y. */
}

void        free_struct_model(STRUCTMODEL sm) 
{
  /* Frees the memory of model. */
  /* if(sm.w) free(sm.w); */ /* this is free'd in free_model */
  if(sm.svm_model) free_model(sm.svm_model,1);
  /* add free calls for user defined data here */
}

void        free_struct_sample(SAMPLE s)
{
  /* Frees the memory of sample s. */
  int i;
  for(i=0;i<s.n;i++) { 
    free_pattern(s.examples[i].x);
    free_label(s.examples[i].y);
  }
  free(s.examples);
}

void        print_struct_help()
{
  /* Prints a help text that is appended to the common help text of
     svm_struct_learn. */
  printf("         --* string  -> custom parameters that can be adapted for struct\n");
  printf("                        learning. The * can be replaced by any character\n");
  printf("                        and there can be multiple options starting with --.\n");
}

void         parse_struct_parameters(STRUCT_LEARN_PARM *sparm)
{
  /* Parses the command line parameters that start with -- */
  int i;

  for(i=0;(i<sparm->custom_argc) && ((sparm->custom_argv[i])[0] == '-');i++) {
    switch ((sparm->custom_argv[i])[2]) 
      { 
      case 'a': i++; /* strcpy(learn_parm->alphafile,argv[i]); */ break;
      case 'e': i++; /* sparm->epsilon=atof(sparm->custom_argv[i]); */ break;
      case 'k': i++; /* sparm->newconstretrain=atol(sparm->custom_argv[i]); */ break;
      default: printf("\nUnrecognized option %s!\n\n",sparm->custom_argv[i]);
	       exit(0);
      }
  }
}

void        print_struct_help_classify()
{
  /* Prints a help text that is appended to the common help text of
     svm_struct_classify. */
  printf("         --* string -> custom parameters that can be adapted for struct\n");
  printf("                       learning. The * can be replaced by any character\n");
  printf("                       and there can be multiple options starting with --.\n");
}

void         parse_struct_parameters_classify(STRUCT_LEARN_PARM *sparm)
{
  /* Parses the command line parameters that start with -- for the
     classification module */
  int i;

  for(i=0;(i<sparm->custom_argc) && ((sparm->custom_argv[i])[0] == '-');i++) {
    switch ((sparm->custom_argv[i])[2]) 
      { 
      /* case 'x': i++; strcpy(xvalue,sparm->custom_argv[i]); break; */
      default: printf("\nUnrecognized option %s!\n\n",sparm->custom_argv[i]);
	       exit(0);
      }
  }
}

