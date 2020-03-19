#include <iomanip>

#include <multibooster/multibooster.h>
#include <matplotlib_interface/matplotlib_interface.h>
#include <track_manager_cached_descriptors.h>

using namespace std;
using namespace Eigen;
using namespace track_manager;

string usageString()
{
  ostringstream oss;
  oss << "Usage: projected_example TMCD DESCRIPTOR_ID OUTPUT_DIR " << endl;
  return oss.str();
}

class SimpleTrack
{
public:
  std::vector<Object*> frames_;
  SimpleTrack copy() const;
};

SimpleTrack SimpleTrack::copy() const
{
  SimpleTrack tr;
  tr.frames_.resize(frames_.size());
  for(size_t i = 0; i < frames_.size(); ++i)
    tr.frames_[i] = new Object(*frames_[i]);

  return tr;
}

// vec must be the right size.
void cat(const Object& obj, VectorXd* vec)
{
  vec->setZero();
  int idx = 0;
  for(size_t i = 0; i < obj.descriptors_.size(); ++i) { 
    int nd = obj.descriptors_[i].vector->rows();
    vec->segment(idx, nd) = obj.descriptors_[i].vector->cast<double>();
    idx += nd;
  }
}


SimpleTrack projectTrack(const TrackCachedDescriptors& tcd, int descr_id, const MatrixXd& projector)
{
  int num_dim = 0;
  for(size_t i = 0; i < tcd.frame_descriptors_[0]->descriptors_.size(); ++i)
    num_dim += tcd.frame_descriptors_[0]->descriptors_[i].vector->rows();
  VectorXd buffer(num_dim);
    
  SimpleTrack tr;
  tr.frames_.resize(tcd.frame_descriptors_.size());
  for(size_t i = 0; i < tcd.frame_descriptors_.size(); ++i) {
    Object* obj = new Object();
    obj->label_ = tcd.frame_descriptors_[i]->label_;
    obj->descriptors_.resize(1);
    if(descr_id != -1)
      obj->descriptors_[0].vector = new VectorXf((projector * tcd.frame_descriptors_[i]->descriptors_[descr_id].vector->cast<double>()).cast<float>());
    else {
      cat(*tcd.frame_descriptors_[i], &buffer);
      obj->descriptors_[0].vector = new VectorXf((projector * buffer).cast<float>());
    }
    obj->descriptors_[0].length_squared = obj->descriptors_[0].vector->norm();

    tr.frames_[i] = obj;
  }
  
  return tr; 
}

MultiBoosterDataset* tracksToDataset(const vector<SimpleTrack>& tracks)
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


void classify(MultiBooster& mb, const SimpleTrack& track, double* confidence, int* label)
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
			 const vector<SimpleTrack>& seed,
			 const vector<SimpleTrack>& unlabeled,
			 const string& filename)
{
  vector<SimpleTrack> all;
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
	  mpli("scatter(x, y, marker='x', color='#700000', zorder=1)");
	else
	  mpli("scatter(x, y, marker='x', color='black', zorder=1)");
      }
      else if(all[i].frames_[j]->label_ == 0) {
	if(correct)
	  mpli("scatter(x, y, marker='o', c='#484848', zorder=2)");
	else
	  mpli("scatter(x, y, marker='o', c='white', zorder=2)");
      }
    }
  }
  cout << "Accuracy: " << num_correct << " / " << all.size() << " = " << (double)num_correct / (double)all.size() << endl;
  
  // -- Save.
  mpli("xlim(-2.5, 4)");
  mpli("ylim(-3.5, 5)");
  mpli("draw()");
  mpliExport(filename);
  mpli("savefig(filename + '.png')");
  mpli("savefig(filename + '.pdf')");
  mpli("clf()");
}


void tbssl(const string& output_dir, const string& run_id, const vector<SimpleTrack>& seed, const vector<SimpleTrack>& unlabeled)
{

  double thresh = 1.0;
  vector<SimpleTrack> working;
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
    oss << output_dir << "/" << run_id << "_classification_epoch_" << setfill('0') << setw(3) << epoch;
    drawClassifications(mb, seed, unlabeled, oss.str());
    
    if(working.size() <= prev_working_size)
      break;

    ++epoch;
  }
}

void updatexxt(const VectorXd& buffer, const VectorXd& mean, MatrixXd* xxt)
{
  VectorXd z = buffer - mean;
  for(int i = 0; i < z.rows(); ++i)
    for(int j = 0; j < z.rows(); ++j)
      xxt->coeffRef(i, j) += z(i) * z(j);
}

int main(int argc, char** argv)
{
  if(argc != 4) {
    cout << usageString() << endl;
    return 0;
  }
  
  string tmcd_path = argv[1];
  int descr_id = atoi(argv[2]);
  string output_dir = argv[3];

  assert(descr_id == -1);
  
  // -- Load tracks.
  cout << "Loading " << tmcd_path << endl;
  TrackManagerCachedDescriptors tmcd(tmcd_path);
  int num_dim;
  if(descr_id == -1) {
    cout << "Concatenating all descriptors." << endl;
    num_dim = 0;
    for(size_t i = 0; i < tmcd.tracks_[0]->frame_descriptors_[0]->descriptors_.size(); ++i)
      num_dim += tmcd.tracks_[0]->frame_descriptors_[0]->descriptors_[i].vector->rows();
  }
  else
    num_dim = tmcd.tracks_[0]->frame_descriptors_[0]->descriptors_[descr_id].vector->rows();
  
  cout << "Descriptor space " << descr_id << " has " << num_dim << " dimensions." << endl;

  // -- Compute the mean.
  VectorXd mean = VectorXd::Zero(num_dim);
  double num_pts = 0;
  VectorXd buffer(num_dim);
  for(size_t i = 0; i < tmcd.tracks_.size(); ++i) { 
    for(size_t j = 0; j < tmcd.tracks_[i]->frame_descriptors_.size(); ++j) { 
      cat(*tmcd.tracks_[i]->frame_descriptors_[j], &buffer);
      mean += buffer;
      //mean += tmcd.tracks_[i]->frame_descriptors_[j]->descriptors_[descr_id].vector->cast<double>();
      ++num_pts;
    }
  }
  mean /= num_pts;
  
  // -- Compute XX^T, with mean subtracted off.
  cout << "Computing XX^T." << endl;
  MatrixXd xxt = MatrixXd::Zero(num_dim, num_dim);
  for(size_t i = 0; i < tmcd.tracks_.size(); ++i) {
    cout << "Track " << i << " / " << tmcd.tracks_.size() << endl;
    cat(*tmcd.tracks_[i]->frame_descriptors_[0], &buffer);
    updatexxt(buffer, mean, &xxt);
    
//     for(size_t j = 0; j < tmcd.tracks_[i]->frame_descriptors_.size(); ++j) {
//       cat(*tmcd.tracks_[i]->frame_descriptors_[j], &buffer);
//       //xxt += (buffer - mean) * (buffer - mean).transpose();
//       updatexxt(buffer, mean, &xxt);
//     }
  }

  // -- Compute SVD of XX^T, project to 2D.
  cout << "Computing SVD" << endl;
  JacobiSVD<MatrixXd> svd(xxt, ComputeFullU);
  VectorXd sing = svd.singularValues();
  cout << "Top 3 singular values: " << sing.head(3).transpose() << endl;
  MatrixXd U = svd.matrixU();

  MatrixXd projector(2, U.rows());
  projector.row(0) = U.col(0).normalized().transpose();
  projector.row(1) = U.col(1).normalized().transpose();

  cout << "Projecting" << endl;
  vector<SimpleTrack> tracks(tmcd.tracks_.size());
  for(size_t i = 0; i < tracks.size(); ++i) {
    if(tmcd.tracks_[i]->label() == -2)
      continue;
    tracks[i] = projectTrack(*tmcd.tracks_[i], descr_id, projector);
  }

  // -- Plot.
  cout << "Plotting." << endl;
  mpliBegin();
  mpli("from pylab import *");
  mpliPrintSize();
  
  mpli("fig = plt.figure()");
  mpli("ax = fig.add_subplot(111)");

  // -- Draw background points.
  int num_bg = 0;
  for(size_t i = 0; i < tracks.size(); ++i) {
    if(tracks[i].frames_[0]->label_ != 0)
      num_bg += tracks[i].frames_.size();
  }

  VectorXd bg_xs(num_bg);
  VectorXd bg_ys(num_bg);
  int idx = 0;
  for(size_t i = 0; i < tracks.size(); ++i) {
    if(tracks[i].frames_[0]->label_ == 0)
      continue;
    for(size_t j = 0; j < tracks[i].frames_.size(); ++j, ++idx) {
      bg_xs(idx) = tracks[i].frames_[j]->descriptors_[0].vector->coeff(0);
      bg_ys(idx) = tracks[i].frames_[j]->descriptors_[0].vector->coeff(1);
    }
  }
	
  mpliExport(bg_xs);
  mpliExport(bg_ys);
  mpli("scatter(bg_xs, bg_ys, marker='x', zorder=0)");


  // -- Draw track paths for cars.
  for(size_t i = 0; i < tracks.size(); ++i) {
    VectorXd path_x(tracks[i].frames_.size());
    VectorXd path_y(tracks[i].frames_.size());
    for(size_t j = 0; j < tracks[i].frames_.size(); ++j) {
      path_x(j) = tracks[i].frames_[j]->descriptors_[0].vector->coeff(0);
      path_y(j) = tracks[i].frames_[j]->descriptors_[0].vector->coeff(1); 
    }

    if(tracks[i].frames_[0]->label_ == 0) { 
      mpliExport(path_x);
      mpliExport(path_y);
      mpli("l0 = Line2D(path_x, path_y, linewidth=1, linestyle=':', marker='None', color='red', zorder=1)");
      mpli("l1 = Line2D(path_x, path_y, linewidth=0.5, linestyle='None', marker='o', color='red', zorder=2)");
      mpli("ax.add_line(l0)");
      mpli("ax.add_line(l1)");
    }
  }

 
//   mpli("xlim(min_x, max_x)");
//   mpli("ylim(min_y, max_y)");
  mpli("draw()");
  mpliExport(output_dir);
  mpli("savefig(output_dir + '/track_structure.png')");
  mpli("savefig(output_dir + '/track_structure.pdf')");
  mpli("clf()");

  /****************************************
   * TBSSL 
   ****************************************/

//   vector<SimpleTrack> seed;
//   vector<SimpleTrack> unlabeled;
//   tbssl(output_dir, "tbssl", seed, unlabeled);
  
  
  return 0;
}


