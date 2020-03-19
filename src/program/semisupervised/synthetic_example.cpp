#include <iomanip>

#include <multibooster/multibooster.h>
#include <matplotlib_interface/matplotlib_interface.h>


using namespace std;
using namespace Eigen;

string usageString()
{
  ostringstream oss;
  oss << "Usage: synthetic_example DIR " << endl;
  return oss.str();
}

class Track
{
public:
  std::vector<Object*> frames_;
  Track copy() const;
};

Track Track::copy() const
{
  Track tr;
  tr.frames_.resize(frames_.size());
  for(size_t i = 0; i < frames_.size(); ++i)
    tr.frames_[i] = new Object(*frames_[i]);

  return tr;
}

int weightedSample(Eigen::VectorXd weights)
{
  weights /= weights.sum();
  double r = (double)rand() / (double)RAND_MAX;

  double cumulative = 0;
  for(int i = 0; i < weights.rows(); ++i) {
    cumulative += weights(i);
    if(cumulative >= r)
      return i;
  }
  assert(0);
}

double gaussianSample(double mean, double sigma)
{
  double val = 0.0;
  for(int i = 0; i < 12; ++i)
    val += ((double)rand() / (double)RAND_MAX - 0.5) * 2.0 * sigma;
  val /= 2.0;
  return val;
}

//! http://en.wikipedia.org/wiki/Multivariate_normal_distribution#Drawing_values_from_the_distribution
VectorXd multivariateGaussianSample(const Vector2d& mean, const Matrix2d& covariance)
{
  VectorXd z(mean.rows());
  for(int i = 0; i < z.rows(); ++i)
    z(i) = gaussianSample(0.0, 1.0);

  Matrix2d A;
  LLT<Matrix2d> llt;
  llt.compute(covariance);

  return mean + llt.matrixL() * z;
}

//! Represents a class which is sampled from a mixture of Gaussians / Markov model.
class SyntheticClass
{
public:
  int label_;
  //! means_[i] is the mean vector for Gaussian i.
  std::vector<Eigen::Vector2d> means_;
  //! covariances_[i] is the covariance matrix for Gaussian i.
  std::vector<Eigen::Matrix2d> covariances_;
  //! probability of starting with a given gaussian.
  Eigen::VectorXd gaussian_weights_;
  //! transition_(i, j) is the probability of switching from Gaussian i to Gaussian j.
  Eigen::MatrixXd transition_;
  //! Unnormalized probability that this class will be sampled relative to other classes.
  double weight_;

  Track sampleTrack(int length) const;
  int sampleNewCluster(int gid) const;
};

int SyntheticClass::sampleNewCluster(int gid) const
{
  return weightedSample(transition_.row(gid));
}

Track SyntheticClass::sampleTrack(int length) const
{
  Track tr;
  tr.frames_.resize(length);

  int gid = weightedSample(gaussian_weights_);
  for(size_t i = 0; i < tr.frames_.size(); ++i) {
    
    Object* obj = new Object();
    obj->label_ = label_;
    obj->descriptors_.resize(1);
    obj->descriptors_[0].vector = new Eigen::VectorXf(2);
    *obj->descriptors_[0].vector = multivariateGaussianSample(means_[gid], covariances_[gid]).cast<float>();
    obj->descriptors_[0].length_squared = obj->descriptors_[0].vector->norm();
    
    tr.frames_[i] = obj;

    gid = sampleNewCluster(gid);
  }

  return tr;
}

class SyntheticTrackGenerator
{
public:
  std::vector<SyntheticClass> classes_;
  Track sampleTrack(int length) const;
};

Track SyntheticTrackGenerator::sampleTrack(int length) const
{
  VectorXd weights(classes_.size());
  for(size_t i = 0; i < classes_.size(); ++i)
    weights(i) = classes_[i].weight_;
  
  int idx = weightedSample(weights);
  return classes_[idx].sampleTrack(length);
}

// Assumes that all tracks are the same length.
MultiBoosterDataset* tracksToDataset(const vector<Track>& tracks)
{
  vector<string> classes; 
  classes.push_back("car");
  vector<string> descriptor_spaces;
  descriptor_spaces.push_back("default");
  MultiBoosterDataset* mbd = new MultiBoosterDataset(classes, descriptor_spaces);
  vector<Object*> objects;
  objects.reserve(tracks.size() * tracks[0].frames_.size());
  for(size_t i = 0; i < tracks.size(); ++i) {
    for(size_t j = 0; j < tracks[i].frames_.size(); ++j) { 
      objects.push_back(new Object(*tracks[i].frames_[j]));
    }
  }
  mbd->setObjs(objects);
  return mbd;
}

void classify(MultiBooster& mb, const Track& track, double* confidence, int* label)
{
  VectorXf response = VectorXf::Zero(mb.prior_.rows());
  for(size_t i = 0; i < track.frames_.size(); ++i) {
    assert(!track.frames_[i]->descriptors_.empty());
    assert(track.frames_[i]->descriptors_[0].vector);
    response += mb.classify(*track.frames_[i]) - mb.prior_;
  }

  response /= (double)track.frames_.size();
  response += mb.prior_;

  *confidence = response.maxCoeff(label);
  if(*confidence <= 0) { 
    *confidence *= -1;
    *label = -1;
  }
}

// filename should not contain extension.
void drawClassifications(MultiBooster& mb,
			 const vector<Track>& seed,
			 const vector<Track>& unlabeled,
			 const string& filename)
{
  vector<Track> all;
  all.insert(all.end(), seed.begin(), seed.end());
  all.insert(all.end(), unlabeled.begin(), unlabeled.end());


  mpli("fig = plt.figure()");
  mpli("ax = fig.add_subplot(111)");

  int num_correct = 0;
  for(size_t i = 0; i < all.size(); ++i) {
    double confidence;
    int label;
    classify(mb, all[i], &confidence, &label);
    bool correct = (label == all[i].frames_[0]->label_);
    if(correct)
      ++num_correct;
    
    for(size_t j = 0; j < all[i].frames_.size(); ++j) {
      mpliNamedExport("x", all[i].frames_[j]->descriptors_[0].vector->coeff(0));
      mpliNamedExport("y", all[i].frames_[j]->descriptors_[0].vector->coeff(1));
      if(all[i].frames_[j]->label_ == -1) {
	if(correct)
	  mpli("scatter(x, y, marker='x', color='black', zorder=1)");
	else
	  mpli("scatter(x, y, marker='x', color='red', s=60, zorder=1)");
      }
      else if(all[i].frames_[j]->label_ == 0) {
	if(correct)
	  mpli("scatter(x, y, marker='o', c='green', zorder=2)");
	else
	  mpli("scatter(x, y, marker='o', c='white', zorder=2)");
      }
    }
  }
  cout << "Accuracy: " << num_correct << " / " << all.size() << " = " << (double)num_correct / (double)all.size() << endl;
  
  // -- Save.
  mpli("xlim(-2.5, 4)");
  mpli("ylim(-3.5, 5)");
  mpli("ax.get_xaxis().set_visible(False)");
  mpli("ax.get_yaxis().set_visible(False)");
  mpli("draw()");
  mpliExport(filename);
  mpli("savefig(filename + '.png')");
  mpli("savefig(filename + '.pdf')");
  mpli("clf()");
}

void tbssl(const string& dir, const string& run_id, const vector<Track>& seed, const vector<Track>& unlabeled)
{

  double thresh = 1.0;
  vector<Track> working;
  working = seed;
  size_t prev_working_size = 0;
  int epoch = 0;
  while(true) {
    cout << "Epoch " << epoch << endl;
    MultiBoosterDataset* mbd = tracksToDataset(working);
    MultiBooster mb(mbd);
    mb.verbose_ = false;
    mb.train(20, 0, 500);
    delete mbd;

    prev_working_size = working.size();
    working = seed;
    
    for(size_t i = 0; i < unlabeled.size(); ++i) {
      double confidence;
      int label;
      classify(mb, unlabeled[i], &confidence, &label);
      if(label != -1 && confidence >= thresh) {
	working.push_back(unlabeled[i].copy());
	for(size_t j = 0; j < working.back().frames_.size(); ++j)
	  working.back().frames_[j]->label_ = label;
      }
    }
    
    ostringstream oss;
    oss << dir << "/" << run_id << "_classification_epoch_" << setfill('0') << setw(3) << epoch;
    drawClassifications(mb, seed, unlabeled, oss.str());
    
    if(working.size() <= prev_working_size)
      break;

    ++epoch;
  }
}

int main(int argc, char** argv)
{
  if(argc != 2) {
    cout << usageString() << endl;
    return 0;
  }

  string dir = argv[1];
  cout << "Saving to " << dir << endl;
  
  /****************************************
   * Dataset setup
   ****************************************/
  
  SyntheticClass background;
  background.label_ = -1;
  background.means_.push_back(Vector2d::Zero());
  background.covariances_.push_back(Matrix2d::Identity() * 10);
  background.gaussian_weights_ = VectorXd::Ones(1);
  background.transition_ = MatrixXd::Ones(1, 1);
  background.weight_ = .5;

  SyntheticClass car;
  car.label_ = 0;
  Vector2d mean;
  mean(0) = -1.5; mean(1) = -0.5;
  car.means_.push_back(mean);
  mean(0) = 2.0; mean(1) = 2.0;
  car.means_.push_back(mean);
  mean(0) = 0.5; mean(1) = -2.25;
  car.means_.push_back(mean);

  Matrix2d cov;
  cov(0, 0) = 1;   cov(0, 1) = 1.5;
  cov(1, 0) = -0.5; cov(1, 1) = 1;
  cov *= 0.1;
  car.covariances_.push_back(cov);
  cov(0, 0) = 1;   cov(0, 1) = -1;
  cov(1, 0) = -2; cov(1, 1) = 1;
  cov *= 0.3;
  car.covariances_.push_back(cov);
  cov(0, 0) = 1;   cov(0, 1) = -0.5;
  cov(1, 0) = 0.5; cov(1, 1) = 1;
  cov *= 0.1;
  car.covariances_.push_back(cov);

  car.gaussian_weights_ = VectorXd(3);
  car.gaussian_weights_(0) = 0.5;
  car.gaussian_weights_(1) = 0.25;
  car.gaussian_weights_(2) = 0.25;

  car.transition_ = MatrixXd::Identity(3, 3) * 0.9;
  car.transition_(0, 1) = 0.1;
  car.transition_(1, 2) = 0.1;
  car.transition_(2, 1) = 0.1;
    
  car.weight_ = .5;

  SyntheticTrackGenerator stg;
  stg.classes_.push_back(background);
  stg.classes_.push_back(car);

  // -- Sample "unlabeled" dataset for SSL.
  int num_unlabeled = 300;
  int num_frames = 5;
  vector<Track> unlabeled;
  unlabeled.resize(num_unlabeled);
  for(size_t i = 0; i < unlabeled.size(); ++i)
    unlabeled[i] = stg.sampleTrack(num_frames);
  
  // -- Sample seed dataset.
  Track seed_car = car.sampleTrack(num_frames);
  vector<Track> seed;
  seed.push_back(seed_car);
  int num_seed_background = 50;
  for(int i = 0; i < num_seed_background; ++i)
    seed.push_back(background.sampleTrack(num_frames));

		   
  /****************************************
   * Track structure plot.
   ****************************************/
  
  // -- Collect data for plotting unlabeled tracks.
  int num_bg = 0;
  int num_car = 0;
  for(size_t i = 0; i < unlabeled.size(); ++i) {
    if(unlabeled[i].frames_[0]->label_ == -1)
      num_bg += unlabeled[i].frames_.size();
    else
      num_car += unlabeled[i].frames_.size();
  }

  VectorXd background_pts_x(num_bg);
  VectorXd background_pts_y(num_bg);
  int idx = 0;
  for(size_t i = 0; i < unlabeled.size(); ++i) {
    if(unlabeled[i].frames_[0]->label_ != -1)
      continue;

    for(size_t j = 0; j < unlabeled[i].frames_.size(); ++j, ++idx) { 
      background_pts_x(idx) = unlabeled[i].frames_[j]->descriptors_[0].vector->coeff(0);
      background_pts_y(idx) = unlabeled[i].frames_[j]->descriptors_[0].vector->coeff(1);
    }
  }

  VectorXd car_pts_x(num_car);
  VectorXd car_pts_y(num_car);
  idx = 0;
  for(size_t i = 0; i < unlabeled.size(); ++i) {
    if(unlabeled[i].frames_[0]->label_ != 0)
      continue;

    for(size_t j = 0; j < unlabeled[i].frames_.size(); ++j, ++idx) { 
      car_pts_x(idx) = unlabeled[i].frames_[j]->descriptors_[0].vector->coeff(0);
      car_pts_y(idx) = unlabeled[i].frames_[j]->descriptors_[0].vector->coeff(1);
    }
  }

  // -- Draw points for background and car examples in the unlabeled set.
  mpliBegin();
  mpli("from pylab import *");
  mpliPrintSize();

 
  mpli("fig = plt.figure()");
  mpli("ax = fig.add_subplot(111)");    
  mpliExport(background_pts_x);
  mpliExport(background_pts_y);
  mpli("scatter(background_pts_x, background_pts_y, marker='x')");
  mpliExport(car_pts_x);
  mpliExport(car_pts_y);
  mpli("scatter(car_pts_x, car_pts_y, marker='o', s=20, c='white')");

  // -- Draw lines for tracks of cars in the unlabeled set.
  VectorXd xdata(num_frames);
  VectorXd ydata(num_frames);
  for(size_t i = 0; i < unlabeled.size(); ++i) {
    if(unlabeled[i].frames_[0]->label_ != 0)
      continue;
    
    for(size_t j = 0; j < unlabeled[i].frames_.size(); ++j) {
      xdata(j) = unlabeled[i].frames_[j]->descriptors_[0].vector->coeff(0);
      ydata(j) = unlabeled[i].frames_[j]->descriptors_[0].vector->coeff(1);

      if(j > 0) { 
	mpliNamedExport("x", xdata(j-1));
	mpliNamedExport("y", ydata(j-1));
	mpliNamedExport("dx", xdata(j) - xdata(j-1));
	mpliNamedExport("dy", ydata(j) - ydata(j-1));
	mpli("arrow(x, y, dx, dy, linestyle='dotted', linewidth=0.5, zorder=0)");
      }
    }
  }

  // -- Draw seed background labels.
  for(size_t i = 1; i < seed.size(); ++i) {
    VectorXd seed_bg_x(seed[i].frames_.size());
    VectorXd seed_bg_y(seed[i].frames_.size());
    for(size_t j = 0; j < seed[i].frames_.size(); ++j) { 
      seed_bg_x(j) = seed[i].frames_[j]->descriptors_[0].vector->coeff(0);
      seed_bg_y(j) = seed[i].frames_[j]->descriptors_[0].vector->coeff(1);
    }
    mpliExport(seed_bg_x);
    mpliExport(seed_bg_y);
    mpli("scatter(seed_bg_x, seed_bg_y, marker='x', s=60, color='black')");
  }
  
  // -- Draw seed car labels.
  VectorXd seed_car_x(num_frames);
  VectorXd seed_car_y(num_frames);
  for(size_t i = 0; i < seed_car.frames_.size(); ++i) {
    seed_car_x(i) = seed_car.frames_[i]->descriptors_[0].vector->coeff(0);
    seed_car_y(i) = seed_car.frames_[i]->descriptors_[0].vector->coeff(1);
  }
  mpliExport(seed_car_x);
  mpliExport(seed_car_y);
  mpli("scatter(seed_car_x, seed_car_y, marker='o', s=40, c='green')");

  // -- Save.
  mpli("xlim(-2.5, 4)");
  mpli("ylim(-3.5, 5)");
  mpli("ax.get_xaxis().set_visible(False)");
  mpli("ax.get_yaxis().set_visible(False)");
  mpli("draw()");
  mpliExport(dir);
  mpli("savefig(dir + '/track_structure.png')");
  mpli("savefig(dir + '/track_structure.pdf')");
  mpli("clf()");


  /****************************************
   * TBSSL and self-learning experiments
   ****************************************/

  cout << "============================================================" << endl;
  cout << "= TBSSL" << endl;
  cout << "============================================================" << endl;
  tbssl(dir, "tbssl", seed, unlabeled);

  cout << "============================================================" << endl;
  cout << "= Self-learning" << endl;
  cout << "============================================================" << endl;
  
  // -- Make length-1 tracks for seed and unlabeled data.
  vector<Track> sl_seed(seed.size() * num_frames);
  for(size_t i = 0, idx = 0; i < seed.size(); ++i)
    for(size_t j = 0; j < seed[i].frames_.size(); ++j, ++idx)
      sl_seed[idx].frames_.push_back(seed[i].frames_[j]);

  vector<Track> sl_unlabeled(unlabeled.size() * num_frames);
  for(size_t i = 0, idx = 0; i < unlabeled.size(); ++i)
    for(size_t j = 0; j < unlabeled[i].frames_.size(); ++j, ++idx)
      sl_unlabeled[idx].frames_.push_back(unlabeled[i].frames_[j]);

  tbssl(dir, "self-learning", sl_seed, sl_unlabeled);
}
