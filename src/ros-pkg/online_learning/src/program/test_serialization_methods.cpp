#include <serializable/serializable.h>
#include <eigen_extensions/eigen_extensions.h>
#include <bag_of_tricks/high_res_timer.h>

using namespace std;
using namespace Eigen;

#define NUM_VECTORS (getenv("NUM_VECTORS") ? atoi(getenv("NUM_VECTORS")) : 10000)
#define NUM_ROWS (getenv("NUM_ROWS") ? atoi(getenv("NUM_ROWS")) : 10000)

class Cohesive : public Serializable
{
public:
  double* data_;
  int num_;

  Cohesive(int rows, int num_vectors);
  ~Cohesive();
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
  std::string status() const;
};

Cohesive::Cohesive(int rows, int num_vectors) :
  data_(NULL),
  num_(rows * num_vectors)
{
  data_ = (double*)malloc(num_ * sizeof(double));
  for(int i = 0; i < num_; ++i)
    data_[i] = (double)rand() / (double)RAND_MAX;
}

Cohesive::~Cohesive()
{
  if(data_)
    free(data_);
}

void Cohesive::serialize(ostream& out) const
{
  assert(data_);
  assert(num_ > 0);
  out.write((char*)&num_, sizeof(num_));
  out.write((char*)data_, num_ * sizeof(double));
}

void Cohesive::deserialize(istream& in)
{
  if(data_)
    free(data_);

  in.read((char*)&num_, sizeof(num_));
  data_ = (double*)malloc(num_ * sizeof(double));
  in.read((char*)data_, num_ * sizeof(double));
}

string Cohesive::status() const
{
  ostringstream oss;
  for(int i = 0; i < num_; ++i)
    oss << data_[i] << " ";
  oss << endl;
  return oss.str();
}

class Fragmented : public Serializable
{
public:
  double** data_;
  int rows_;
  int num_vectors_;

  Fragmented(int rows, int num_vectors);
  ~Fragmented();
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
  std::string status() const;
};

Fragmented::Fragmented(int rows, int num_vectors) :
  rows_(rows),
  num_vectors_(num_vectors)
{
  data_ = (double**)malloc(num_vectors_ * sizeof(double*));
  for(int i = 0; i < num_vectors_; ++i) { 
    data_[i] = (double*)malloc(rows_ * sizeof(double));
    for(int j = 0; j < rows_; ++j)
      data_[i][j] = (double)rand() / (double)RAND_MAX;
  }
}

Fragmented::~Fragmented()
{
  if(data_) {
    for(int i = 0; i < num_vectors_; ++i)
      free(data_[i]);
    free(data_);
  }
}

void Fragmented::serialize(ostream& out) const
{
  assert(data_);
  out.write((char*)&num_vectors_, sizeof(int));
  out.write((char*)&rows_, sizeof(int));
  for(int i = 0; i < num_vectors_; ++i)
    out.write((char*)data_[i], rows_ * sizeof(double));
}

void Fragmented::deserialize(istream& in)
{
  if(data_) {
    for(int i = 0; i < num_vectors_; ++i)
      free(data_[i]);
    free(data_);
  }

  in.read((char*)&num_vectors_, sizeof(int));
  in.read((char*)&rows_, sizeof(int));
  data_ = (double**)malloc(num_vectors_ * sizeof(double*));
  for(int i = 0; i < num_vectors_; ++i) { 
    data_[i] = (double*)malloc(rows_ * sizeof(double));
    in.read((char*)data_[i], rows_ * sizeof(double));
  }
}

string Fragmented::status() const
{
  ostringstream oss;
  for(int i = 0; i < num_vectors_; ++i)
    for(int j = 0; j < rows_; ++j)
      oss << data_[i][j] << " ";
  oss << endl;
  return oss.str();
}

class EigenCohesive : public Serializable
{
public:
  Eigen::MatrixXd data_;

  EigenCohesive(int rows, int num_vectors);
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
  std::string status() const;
};

EigenCohesive::EigenCohesive(int rows, int num_vectors)
{
  data_ = MatrixXd::Random(rows, num_vectors);
}

void EigenCohesive::serialize(std::ostream& out) const
{
  eigen_extensions::serialize(data_, out);
}

void EigenCohesive::deserialize(std::istream& in)
{
  eigen_extensions::deserialize(in, &data_);
}

std::string EigenCohesive::status() const
{
  ostringstream oss;
  oss << data_ << endl;
  return oss.str();
}

class EigenFragmented : public Serializable
{
public:
  std::vector<Eigen::VectorXd> data_;
  
  EigenFragmented(int rows, int num_vectors);
  void serialize(std::ostream& out) const;
  void deserialize(std::istream& in);
  std::string status() const;
};

EigenFragmented::EigenFragmented(int rows, int num_vectors)
{
  data_.resize(num_vectors);
  for(size_t i = 0; i < data_.size(); ++i)
    data_[i] = VectorXd::Random(rows);
}

void EigenFragmented::serialize(std::ostream& out) const
{
  size_t buf = data_.size();
  out.write((char*)&buf, sizeof(buf));
  for(size_t i = 0; i < data_.size(); ++i)
    eigen_extensions::serialize(data_[i], out);
}

void EigenFragmented::deserialize(std::istream& in)
{
  size_t buf;
  in.read((char*)&buf, sizeof(buf));
  data_.resize(buf);
  for(size_t i = 0; i < data_.size(); ++i)
    eigen_extensions::deserialize(in, &data_[i]);
}

std::string EigenFragmented::status() const
{
  ostringstream oss;
  for(size_t i = 0; i < data_.size(); ++i)
    oss << data_[i].transpose() << endl;
  return oss.str();
}

int main(int argc, char** argv)
{
  // -- Test serialization for accuracy.
  // EigenCohesive coh1(3, 3);
  // cout << coh1.status() << endl;
  // coh1.save("cohesive");
  // EigenCohesive coh2(1, 1);
  // coh2.load("cohesive");
  // cout << coh2.status() << endl;

  // EigenFragmented frag1(3, 3);
  // cout << frag1.status() << endl;
  // frag1.save("fragmented");
  // EigenFragmented frag2(1, 1);
  // frag2.load("fragmented");
  // cout << frag2.status() << endl;

//   Cohesive coh1(3, 3);
//   cout << coh1.status() << endl;
//   coh1.save("raw_cohesive");
//   Cohesive coh2(1, 1);
//   coh2.load("raw_cohesive");
//   cout << coh2.status() << endl;

//   Fragmented frag1(3, 3);
//   cout << frag1.status() << endl;
//   frag1.save("raw_fragmented");
//   Fragmented frag2(1, 1);
//   frag2.load("raw_fragmented");
//   cout << frag2.status() << endl;

  // -- Run timing tests.
  if(getenv("SERIALIZE")) { 
    {
      EigenFragmented frag(NUM_ROWS, NUM_VECTORS);
      HighResTimer hrt("fragmented eigen serialization");
      hrt.start();
      frag.save("eigen_fragmented");
      hrt.stop();
      cout << hrt.reportSeconds() << endl;
    }
    
    {
      EigenCohesive coh(NUM_ROWS, NUM_VECTORS);
      HighResTimer hrt("cohesive eigen serialization");
      hrt.start();
      coh.save("eigen_cohesive");
      hrt.stop();
      cout << hrt.reportSeconds() << endl;
    }

    {
      Cohesive coh(NUM_ROWS, NUM_VECTORS);
      HighResTimer hrt("cohesive raw serialization");
      hrt.start();
      coh.save("raw_cohesive");
      hrt.stop();
      cout << hrt.reportSeconds() << endl;
    }

    {
      Fragmented frag(NUM_ROWS, NUM_VECTORS);
      HighResTimer hrt("fragmented raw serialization");
      hrt.start();
      frag.save("raw_fragmented");
      hrt.stop();
      cout << hrt.reportSeconds() << endl;
    }
  }

  cout << endl;
  
  {
    EigenFragmented frag(1, 1);
    HighResTimer hrt("fragmented eigen deserialization");
    hrt.start();
    frag.load("eigen_fragmented");
    hrt.stop();
    cout << hrt.reportSeconds() << endl;
  }
  
  {
    EigenCohesive coh(1, 1);
    HighResTimer hrt("cohesive eigen deserialization");
    hrt.start();
    coh.load("eigen_cohesive");
    hrt.stop();
    cout << hrt.reportSeconds() << endl;
  }

  {
    Cohesive coh(1, 1);
    HighResTimer hrt("cohesive raw serialization");
    hrt.start();
    coh.load("raw_cohesive");
    hrt.stop();
    cout << hrt.reportSeconds() << endl;
  }

  {
    Fragmented frag(1, 1);
    HighResTimer hrt("fragmented raw serialization");
    hrt.start();
    frag.load("raw_fragmented");
    hrt.stop();
    cout << hrt.reportSeconds() << endl;
  }
    
  
  return 0;
}
