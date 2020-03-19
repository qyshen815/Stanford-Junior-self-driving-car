#include <online_learning/synthetic_data_generator.h>
#include <online_learning/trainers.h>
#include <bag_of_tricks/bag_of_tricks.h>
#include <gtest/gtest.h>


using namespace Eigen;
using namespace odontomachus;
using namespace std;

TEST(Params, Serialization)
{
  Params p;
  p["foo"] = 23489.43985;
  p["bar"] = 42;
  p["baz"] = 13.13;
  cout << p << endl;
  string filename = "test_params";
  p.save(filename);

  Params q;
  q.load(filename);
  cout << q << endl;
  map<string, double>::iterator it;
  for(it = p.begin(); it != p.end(); ++it) {
    EXPECT_TRUE(q.count(it->first) == 1);
    EXPECT_FLOAT_EQ(q[it->first], it->second);
  }
}

// TEST(Classification, Classification)
// {
//   int num_classes = 3;
  
//   Classification c0;
//   c0.response_ = VectorXf::Zero(num_classes);
//   Classification c1;
//   float k = 2.13;
//   c1.response_ = VectorXf::Ones(num_classes) * k;
//   c1.response_(1) += 0.5;
  
//   c0.serialize(cout);
//   c1.serialize(cout);

//   c0 += c1;
//   c0.serialize(cout);
//   EXPECT_FLOAT_EQ(c0.response_(0), c1.response_(0));

//   Classification c2 = c1 * 10.0;
//   c2.serialize(cout);
//   EXPECT_FLOAT_EQ(c1.response_(1) * 10.0, c2.response_(1));  
// }

TEST(Dataset, Serialization)
{
  int num_descriptor_spaces = 10;
  int num_samples = 100;
  
  Dataset dataset;
  dataset.descriptors_ = MatrixXf::Random(num_descriptor_spaces, num_samples);
  dataset.labels_ = VectorXi::Ones(num_samples) * -1;
  dataset.labels_.head(4) = VectorXi::Ones(4) * 2;
  dataset.class_map_ = getDefaultClassMap();
  dataset.descriptor_map_ = getStubDescriptorMap(num_descriptor_spaces);
  dataset.track_end_flags_ = VectorXi::Zero(num_samples);
  dataset.track_end_flags_(3) = 1;
  dataset.track_end_flags_(dataset.track_end_flags_.rows() - 1) = 1;
  dataset.assertConsistency();
  
  string filename = "test_dataset.od";
  dataset.save(filename);

  Dataset dataset2;
  dataset2.load(filename);
  EXPECT_EQ(dataset2.descriptors_.cols(), dataset.descriptors_.cols());
  EXPECT_EQ(dataset2.descriptors_.rows(), dataset.descriptors_.rows());
  EXPECT_EQ(dataset2.labels_.rows(), dataset.labels_.rows());
  
  for(int i = 0; i < dataset2.descriptors_.cols(); ++i) {
    EXPECT_TRUE(dataset2.labels_(i) == dataset.labels_(i));
    EXPECT_FLOAT_EQ((dataset2.descriptors_.col(i) - dataset.descriptors_.col(i)).norm(), 0.0);
  }

  EXPECT_TRUE(dataset.class_map_ == dataset2.class_map_);
  EXPECT_TRUE(dataset.descriptor_map_ == dataset2.descriptor_map_);

  EXPECT_TRUE(dataset.track_end_flags_.rows() == dataset2.track_end_flags_.rows());
  for(int i = 0; i < dataset2.track_end_flags_.rows(); ++i)
    EXPECT_TRUE(dataset.track_end_flags_(i) == dataset2.track_end_flags_(i));
  
  cout << dataset.status() << endl;
  cout << dataset2.status() << endl;
}

TEST(Dataset, Join)
{
  int num_descriptor_spaces = 10;
  int num_samples = 100;

  Dataset dataset1;
  dataset1.class_map_ = getDefaultClassMap();
  dataset1.descriptor_map_ = getStubDescriptorMap(num_descriptor_spaces);
  dataset1.descriptors_ = MatrixXf::Random(num_descriptor_spaces, num_samples);
  dataset1.labels_ = VectorXi::Ones(num_samples) * -1;
  dataset1.labels_.head(4) = VectorXi::Ones(4) * 2;
  dataset1.track_end_flags_ = VectorXi::Zero(num_samples);
  dataset1.track_end_flags_(3) = 1;
  dataset1.track_end_flags_(dataset1.track_end_flags_.rows() - 1) = 1;
  dataset1.assertConsistency();

  Dataset dataset2;
  dataset2.class_map_ = getDefaultClassMap();
  dataset2.descriptor_map_ = dataset1.descriptor_map_;
  dataset2.descriptors_ = MatrixXf::Random(num_descriptor_spaces, num_samples);
  dataset2.labels_ = VectorXi::Ones(num_samples) * -1;
  dataset2.labels_.head(5) = VectorXi::Zero(5);
  dataset2.track_end_flags_ = VectorXi::Zero(num_samples);
  dataset2.track_end_flags_(4) = 1;
  dataset2.track_end_flags_(dataset2.track_end_flags_.rows() - 1) = 1;
  dataset2.assertConsistency();
  
  cout << dataset1.status() << endl;
  cout << dataset2.status() << endl;
  dataset2 += dataset1;
  dataset2.assertConsistency();
  EXPECT_EQ(dataset2.descriptors_.cols(), num_samples * 2);
  EXPECT_EQ(dataset2.labels_.rows(), num_samples * 2);
  EXPECT_FLOAT_EQ(dataset1.descriptors_(7, 13), dataset2.descriptors_(7, 113));
  cout << dataset2.status() << endl;
}

TEST(Dataset, DescriptorMapping)
{
  int num_descriptor_spaces = 3;
  int num_samples = 5;
  
  Dataset dataset;
  dataset.descriptors_ = MatrixXf::Random(num_descriptor_spaces, num_samples);
  dataset.labels_ = VectorXi::Ones(num_samples) * -1;
  dataset.labels_(3) = 2;
  dataset.class_map_ = getDefaultClassMap();
  dataset.descriptor_map_.addName("foo");
  dataset.descriptor_map_.addName("bar");
  dataset.descriptor_map_.addName("baz");
  
  NameMapping2 dmap;
  dmap.addName("baz");
  dmap.addName("foo");

  Dataset copy = dataset;
  
  dataset.applyNameMapping(dmap, 1);
  cout << copy.descriptors_ << endl;
  cout << endl;
  cout << dataset.descriptors_ << endl;
  EXPECT_TRUE(dataset.class_map_ == copy.class_map_);
  EXPECT_TRUE(dataset.descriptors_.rows() == copy.descriptors_.rows() - 1);
  EXPECT_TRUE(dataset.descriptors_.cols() == copy.descriptors_.cols());

  for(int i = 0; i < dataset.descriptors_.cols(); ++i) {
    EXPECT_FLOAT_EQ(dataset.descriptors_(0, i), copy.descriptors_(2, i));
    EXPECT_FLOAT_EQ(dataset.descriptors_(1, i), copy.descriptors_(0, i));
  }
}

// TEST(Dataset, BigTest)
// {
//   int num_descriptors = 500;
//   int num_samples = 1e6;

//   Dataset dataset;
//   HighResTimer timer;
//   timer.start();
//   dataset.descriptors_ = MatrixXf::Zero(num_descriptors, num_samples);
//   dataset.labels_ = VectorXi::Zero(num_samples);
//   string filename = "test_dataset.od";
//   timer.stop();
//   cout << "Creating dataset took " << timer.getSeconds() << " seconds." << endl;

//   timer.start();
//   dataset.save(filename);
//   timer.stop();
//   cout << "Saving dataset took " << timer.getSeconds() << " seconds." << endl;

//   Dataset dataset2;
//   timer.start();
//   dataset2.load(filename);
//   timer.stop();
//   cout << "Loading dataset took " << timer.getSeconds() << " seconds." << endl;
// }

TEST(ProjectionSlicer, NaiveTrainingSimpleCase)
{
  int num_descriptors = 2;
  int num_classes = 1;

  Dataset::Ptr dataset(new Dataset());
  dataset->descriptors_ = MatrixXf(num_descriptors, 5);
  dataset->labels_ = VectorXi(5);

  // -- Set the range to be 0 to 1 for both descriptor spaces, add a few other samples.
  dataset->labels_(0) = -1;
  dataset->descriptors_(0, 0) = 0;
  dataset->descriptors_(1, 0) = 0;

  dataset->labels_(1) = -1;
  dataset->descriptors_(0, 1) = 1;
  dataset->descriptors_(1, 1) = 1;

  dataset->labels_(2) = 0;
  dataset->descriptors_(0, 2) = 0.5;
  dataset->descriptors_(1, 2) = 0.5;

  dataset->labels_(3) = 0;
  dataset->descriptors_(0, 3) = 0.5;
  dataset->descriptors_(1, 3) = 0.5;

  dataset->labels_(4) = 0;
  dataset->descriptors_(0, 4) = 0.25;
  dataset->descriptors_(1, 4) = 0.5;

  // -- Set up classifier.
  ProjectionSlicer::Ptr slicer(new ProjectionSlicer());
  int num_cells = 1000;
  slicer->initialize(num_classes, num_cells, dataset);

  NaiveTrainer trainer;
  trainer.setSmoothing(1);
  trainer.attachSlicer(slicer);
  trainer.train(dataset);

  // -- Check the prior and smoothing.
  //EXPECT_FLOAT_EQ(trainer.counts_(0), 1003.0);
//  EXPECT_FLOAT_EQ(trainer.total_training_instances_, 2005.0);
  double expected_prior = log(1003.0 / 1002.0);
  EXPECT_FLOAT_EQ(slicer->prior_(0), expected_prior);

  // -- Check some specific instances.
  VectorXf instance(num_descriptors);
  instance(0) = 0.75;
  instance(1) = 0.75;
  EXPECT_FLOAT_EQ(slicer->classify(instance).response_(0), -expected_prior);

  instance(0) = 0.5;
  instance(1) = 0.5;
  EXPECT_FLOAT_EQ(slicer->classify(instance).response_(0), -expected_prior + log(3.0) + log(4.0));

  // Test serialization while we're here.
  string filename = "test.ps";
  slicer->save(filename);
  ProjectionSlicer ps;
  ps.load(filename);
  EXPECT_TRUE(ps.offset_.rows() == slicer->offset_.rows());
  for(int i = 0; i < ps.offset_.rows(); ++i)
    EXPECT_FLOAT_EQ(ps.offset_(i), slicer->offset_(i));
  
  EXPECT_TRUE(ps.prior_.rows() == slicer->prior_.rows());
  for(int i = 0; i < ps.prior_.rows(); ++i)
    EXPECT_FLOAT_EQ(ps.prior_(i), slicer->prior_(i));

  EXPECT_TRUE(ps.projection_weights_.rows() == slicer->projection_weights_.rows());
  EXPECT_TRUE(ps.projection_weights_.cols() == slicer->projection_weights_.cols());
  for(int i = 0; i < ps.projection_weights_.cols(); ++i)
    for(int j = 0; j < ps.projection_weights_.rows(); ++j)
      EXPECT_FLOAT_EQ(ps.projection_weights_(j, i), slicer->projection_weights_(j, i));
  
  EXPECT_TRUE(ps.projections_.size() == slicer->projections_.size());
  for(size_t i = 0; i < ps.projections_.size(); ++i) {
    Projection& p1 = *ps.projections_[i];
    Projection& p2 = *slicer->projections_[i];
    EXPECT_TRUE(p1.cells_.rows() == p2.cells_.rows());
    EXPECT_TRUE(p1.cells_.cols() == p2.cells_.cols());
    for(int j = 0; j < p1.cells_.cols(); ++j)
      for(int k = 0; k < p1.cells_.rows(); ++k)
	EXPECT_FLOAT_EQ(p1.cells_(k, j), p2.cells_(k, j));

    EXPECT_TRUE(p1.fallback_response_.rows() == p2.fallback_response_.rows());
    for(int j = 0; j < p1.fallback_response_.rows(); ++j)
      EXPECT_FLOAT_EQ(p1.fallback_response_(j), p2.fallback_response_(j));
  }
}

TEST(ProjectionSlicer, LogisticTraining)
{
  int num_descriptors = 3;
  int num_classes = 2;
  MatrixXd means(num_descriptors, num_classes);
  means(0, 0) = 10.0;
  means(1, 1) = 5.0;
  means(2, 0) = 2.0;
  means(0, 1) = 1.0;
  means(1, 0) = 4.0;
  means(2, 1) = 7.0;
  double stdev = 1;
  SyntheticDataGenerator sdg(0, means, stdev);

  Dataset::Ptr dataset(new Dataset());
  int num_samples = 1e4;
  sdg.sample(num_samples, &dataset->descriptors_, &dataset->labels_);
  dataset->class_map_ = getDefaultClassMap();
  dataset->track_end_flags_ = VectorXi::Ones(num_samples);
  dataset->descriptor_map_.addName("one");
  dataset->descriptor_map_.addName("two");
  dataset->descriptor_map_.addName("three");
  dataset->assertConsistency();
  
  cout << dataset->status() << endl;

  int num_cells = 1000;
  ProjectionSlicer::Ptr slicer(new ProjectionSlicer());
  slicer->initialize(num_classes, num_cells, dataset);
  LogisticTrainer trainer;
  trainer.attachSlicer(slicer);
  trainer.train(dataset);
  cout << slicer->projections_[0]->cells_ << endl;
  cout << "prior: " << endl;
  cout << slicer->prior_.transpose() << endl;

  {
    double num_correct = 0;
    double total = 0;
    for(int i = 0; i < dataset->descriptors_.cols(); ++i) {
      Classification cl = slicer->classify(dataset->descriptors_.col(i));
      //cout << cl << endl;
      if(cl.getClassId() == dataset->labels_(i))
	++num_correct;
      ++total;
    }
    cout << "Accuracy: " << num_correct << " / " << total << endl;
  }

  
  sdg.sample(1e3, &dataset->descriptors_, &dataset->labels_);
  dataset->track_end_flags_ = VectorXi::Ones(1e3);
  dataset->assertConsistency();
  cout << dataset->status() << endl;

  double num_correct = 0;
  double total = 0;
  for(int i = 0; i < dataset->descriptors_.cols(); ++i) {
    Classification cl = slicer->classify(dataset->descriptors_.col(i));
    //cout << cl << endl;
    if(cl.getClassId() == dataset->labels_(i))
      ++num_correct;
    ++total;
  }
  cout << "Accuracy: " << num_correct << " / " << total << endl;
  
}

TEST(ProjectionSlicer, NaiveTraining)
{
  int num_descriptors = 3;
  int num_classes = 2;
  MatrixXd means(num_descriptors, num_classes);
  means(0, 0) = 10.0;
  means(1, 1) = 5.0;
  means(2, 0) = 2.0;
  means(0, 1) = 1.0;
  means(1, 0) = 4.0;
  means(2, 1) = 7.0;
  double stdev = 1;
  SyntheticDataGenerator sdg(0, means, stdev);

  Dataset::Ptr dataset(new Dataset());
  dataset->class_map_ = getDefaultClassMap();
  dataset->descriptor_map_ = getStubDescriptorMap(num_descriptors);
  int num_samples = 1e4;
  sdg.sample(num_samples, &dataset->descriptors_, &dataset->labels_);
  dataset->track_end_flags_ = VectorXi::Ones(num_samples);
  dataset->assertConsistency();
  
  cout << dataset->status() << endl;

  int num_cells = 1000;
  ProjectionSlicer::Ptr slicer(new ProjectionSlicer());
  slicer->initialize(num_classes, num_cells, dataset);
  NaiveTrainer trainer;
  trainer.setSmoothing(1);
  trainer.attachSlicer(slicer);
  trainer.train(dataset);
  cout << slicer->projections_[0]->cells_ << endl;
  cout << "prior: " << endl;
  cout << slicer->prior_.transpose() << endl;

  {
    double num_correct = 0;
    double total = 0;
    for(int i = 0; i < dataset->descriptors_.cols(); ++i) {
      Classification cl = slicer->classify(dataset->descriptors_.col(i));
      //cout << cl << endl;
      if(cl.getClassId() == dataset->labels_(i))
	++num_correct;
      ++total;
    }
    cout << "Accuracy: " << num_correct << " / " << total << endl;
  }

  
  sdg.sample(1e3, &dataset->descriptors_, &dataset->labels_);
  dataset->track_end_flags_ = VectorXi::Ones(1e3);
  dataset->assertConsistency();
  cout << dataset->status() << endl;

  double num_correct = 0;
  double total = 0;
  for(int i = 0; i < dataset->descriptors_.cols(); ++i) {
    Classification cl = slicer->classify(dataset->descriptors_.col(i));
    //cout << cl << endl;
    if(cl.getClassId() == dataset->labels_(i))
      ++num_correct;
    ++total;
  }
  cout << "Accuracy: " << num_correct << " / " << total << endl;
  
}

TEST(ProjectionSlicer, OnlineNaiveTraining)
{
 int num_descriptors = 3;
  int num_classes = 2;
  MatrixXd means(num_descriptors, num_classes);
  means(0, 0) = 10.0;
  means(1, 1) = 5.0;
  means(2, 0) = 2.0;
  means(0, 1) = 1.0;
  means(1, 0) = 4.0;
  means(2, 1) = 7.0;
  double stdev = 1;
  SyntheticDataGenerator sdg(0, means, stdev);

  Dataset::Ptr dataset(new Dataset());
  dataset->class_map_ = getDefaultClassMap();
  dataset->descriptor_map_ = getStubDescriptorMap(3);
  int num_samples = 1e4;
  sdg.sample(num_samples, &dataset->descriptors_, &dataset->labels_);
  dataset->track_end_flags_ = VectorXi::Ones(num_samples);

  Dataset::Ptr part1(new Dataset());
  Dataset::Ptr part2(new Dataset());
  part1->class_map_ = dataset->class_map_;
  part1->descriptor_map_ = dataset->descriptor_map_;
  part2->class_map_ = dataset->class_map_;
  part2->descriptor_map_ = dataset->descriptor_map_;
  int num1 = dataset->descriptors_.cols() / 2;
  int num2 = dataset->descriptors_.cols() - num1;
  part1->descriptors_ = dataset->descriptors_.block(0, 0, dataset->descriptors_.rows(), num1);
  part1->labels_ = dataset->labels_.head(num1);
  part1->track_end_flags_ = dataset->track_end_flags_.head(num1);
  part1->assertConsistency();
  part2->descriptors_ = dataset->descriptors_.block(0, num1, dataset->descriptors_.rows(), num2);
  part2->labels_ = dataset->labels_.tail(num2);
  part2->track_end_flags_ = dataset->track_end_flags_.tail(num2);
  part2->assertConsistency();
  
  cout << dataset->status() << endl;
  cout << part1->status() << endl;
  cout << part2->status() << endl;
  EXPECT_TRUE(part1->labels_.rows() + part2->labels_.rows() == dataset->labels_.rows());


  int num_cells = 1000;
  ProjectionSlicer::Ptr offline(new ProjectionSlicer());
  offline->initialize(num_classes, num_cells, dataset);
  NaiveTrainer trainer;
  trainer.setSmoothing(1);
  trainer.attachSlicer(offline);
  trainer.train(dataset);

  ProjectionSlicer::Ptr online(new ProjectionSlicer());
  online->initialize(num_classes, num_cells, dataset);
  NaiveTrainer stream_trainer;
  stream_trainer.setSmoothing(1);
  stream_trainer.attachSlicer(online);
  stream_trainer.train(part1);
  stream_trainer.train(part2);

  for(int i = 0; i < online->prior_.rows(); ++i)
    EXPECT_FLOAT_EQ(online->prior_(i), offline->prior_(i));

  Dataset::Ptr test(new Dataset());
  test->class_map_ = dataset->class_map_;
  test->descriptor_map_ = dataset->descriptor_map_;
  sdg.sample(100, &test->descriptors_, &test->labels_);
  for(int i = 0; i < test->descriptors_.cols(); ++i) {
    Classification cl_online = online->classify(test->descriptors_.col(i));
    Classification cl_offline = offline->classify(test->descriptors_.col(i));
//     cout << "========================================" << endl;
//     cout << cl_online << endl;
//     cout << cl_offline << endl;
//     cout << "========================================" << endl;
    for(int j = 0; j < cl_online.response_.rows(); ++j)
      EXPECT_FLOAT_EQ(cl_online.response_(j), cl_offline.response_(j));
  }
}

TEST(ProjectionSlicer, LogisticStochasticTrainer)
{
  int num_descriptors = 3;
  int num_classes = 2;
  MatrixXd means(num_descriptors, num_classes);
  means(0, 0) = 10.0;
  means(1, 1) = 5.0;
  means(2, 0) = 2.0;
  means(0, 1) = 1.0;
  means(1, 0) = 4.0;
  means(2, 1) = 7.0;
  double stdev = 1;
  SyntheticDataGenerator sdg(0, means, stdev);

  Dataset::Ptr dataset(new Dataset());
  dataset->class_map_ = getDefaultClassMap();
  dataset->descriptor_map_ = getStubDescriptorMap(num_descriptors);
  int num_samples = 1e4;
  sdg.sample(num_samples, &dataset->descriptors_, &dataset->labels_);
  dataset->track_end_flags_ = VectorXi::Ones(num_samples);
  dataset->assertConsistency();

  cout << dataset->status() << endl;

  int num_cells = 1000;
  ProjectionSlicer::Ptr slicer(new ProjectionSlicer());
  slicer->initialize(num_classes, num_cells, dataset);
  LogisticStochasticTrainer trainer;
  trainer.attachSlicer(slicer);
  trainer.scheduler_ = new ConstantScheduler(0.1);
  trainer.train(dataset);
  cout << slicer->projections_[0]->cells_ << endl;
  cout << "prior: " << endl;
  cout << slicer->prior_.transpose() << endl;

  {
    double num_correct = 0;
    double total = 0;
    for(int i = 0; i < dataset->descriptors_.cols(); ++i) {
      Classification cl = slicer->classify(dataset->descriptors_.col(i));
      //cout << cl << endl;
      if(cl.getClassId() == dataset->labels_(i))
	++num_correct;
      ++total;
    }
    cout << "Accuracy: " << num_correct << " / " << total << endl;
  }

  
  sdg.sample(1e3, &dataset->descriptors_, &dataset->labels_);
  dataset->track_end_flags_ = VectorXi::Ones(1e3);
  cout << dataset->status() << endl;

  double num_correct = 0;
  double total = 0;
  for(int i = 0; i < dataset->descriptors_.cols(); ++i) {
    Classification cl = slicer->classify(dataset->descriptors_.col(i));
    //cout << cl << endl;
    if(cl.getClassId() == dataset->labels_(i))
      ++num_correct;
    ++total;
  }
  cout << "Accuracy: " << num_correct << " / " << total << endl;
  
}

TEST(ProjectionSlicer, HybridStochasticTrainer)
{
  int num_descriptors = 3;
  int num_classes = 2;
  MatrixXd means(num_descriptors, num_classes);
  means(0, 0) = 10.0;
  means(1, 1) = 5.0;
  means(2, 0) = 2.0;
  means(0, 1) = 1.0;
  means(1, 0) = 4.0;
  means(2, 1) = 7.0;
  double stdev = 1;
  SyntheticDataGenerator sdg(0, means, stdev);

  Dataset::Ptr dataset(new Dataset());
  dataset->class_map_ = getDefaultClassMap();
  dataset->descriptor_map_ = getStubDescriptorMap(num_descriptors);
  int num_samples = 1e4;
  sdg.sample(num_samples, &dataset->descriptors_, &dataset->labels_);
  dataset->track_end_flags_ = VectorXi::Ones(num_samples);
  dataset->assertConsistency();
  cout << dataset->status() << endl;

  int num_cells = 1000;
  ProjectionSlicer::Ptr slicer(new ProjectionSlicer());
  slicer->initialize(num_classes, num_cells, dataset);
  HybridStochasticTrainer trainer;
  trainer.setSmoothing(1);
  trainer.scheduler_ = new ConstantScheduler(0.1);
  trainer.attachSlicer(slicer);
  trainer.train(dataset);
  trainer.trainProjectionWeights(dataset);

  cout << "Projection weights: " << endl;
  cout << slicer->projection_weights_ << endl;
  cout << "prior: " << endl;
  cout << slicer->prior_.transpose() << endl;

  {
    double num_correct = 0;
    double total = 0;
    for(int i = 0; i < dataset->descriptors_.cols(); ++i) {
      Classification cl = slicer->classify(dataset->descriptors_.col(i));
      //cout << cl << endl;
      if(cl.getClassId() == dataset->labels_(i))
	++num_correct;
      ++total;
    }
    cout << "Accuracy: " << num_correct << " / " << total << endl;
  }

  
  sdg.sample(1e3, &dataset->descriptors_, &dataset->labels_);
  dataset->track_end_flags_ = VectorXi::Ones(1e3);
  cout << dataset->status() << endl;

  double num_correct = 0;
  double total = 0;
  for(int i = 0; i < dataset->descriptors_.cols(); ++i) {
    Classification cl = slicer->classify(dataset->descriptors_.col(i));
    //cout << cl << endl;
    if(cl.getClassId() == dataset->labels_(i))
      ++num_correct;
    ++total;
  }
  cout << "Accuracy: " << num_correct << " / " << total << endl;
  
}

TEST(DatasetSplitter, Split)
{
  int num_descriptor_spaces = 10;
  int num_tracks = 983;
  int num_frames_per_track = 7;
  int num_classes = 3;
  int num_instances = num_tracks * num_frames_per_track;
  
  Dataset::Ptr dataset(new Dataset());
  dataset->descriptors_ = MatrixXf::Random(num_descriptor_spaces, num_instances);
  dataset->labels_ = VectorXi::Ones(num_instances) * -1;
  dataset->class_map_ = getDefaultClassMap();
  dataset->descriptor_map_ = getStubDescriptorMap(num_descriptor_spaces);
  dataset->track_end_flags_ = VectorXi::Zero(num_instances);

  for(int i = 0; i < num_tracks; ++i) {
    int label = (rand() % (num_classes+1)) - 1;
    for(int j = 0; j < num_frames_per_track; ++j) {
      int idx = i * num_frames_per_track + j;
      dataset->labels_(idx) = label;
      dataset->track_end_flags_(idx) = 0;
      if(j == num_frames_per_track - 1)
	dataset->track_end_flags_(idx) = 1;
    }
  }
  dataset->assertConsistency();

  VectorXd prob(3);
  prob(0) = 0.1;
  prob(1) = 0.7;
  prob(2) = 0.2;
  DatasetSplitter splitter(prob);
  splitter.splitTrackwise(dataset);

  for(size_t i = 0; i < splitter.partitions_.size(); ++i) { 
    cout << "Partition " << i << ": " << splitter.partitions_[i]->track_end_flags_.sum() << " tracks." << endl;
    cout << splitter.partitions_[i]->status() << endl;
  }

  splitter.splitFramewise(dataset);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
