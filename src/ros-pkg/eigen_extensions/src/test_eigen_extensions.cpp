#include <eigen_extensions/eigen_extensions.h>
#include <gtest/gtest.h>

using namespace std;
using namespace Eigen;

int size = 3;

TEST(EigenExtensions, MatrixXd_serialization) {
  MatrixXd mat = MatrixXd::Random(size, size);
  eigen_extensions::save(mat, "matxd.eig");
  MatrixXd mat2;
  eigen_extensions::load("matxd.eig", &mat2);
  EXPECT_TRUE(mat.isApprox(mat2));
}

TEST(EigenExtensions, MatrixXf_serialization) {
  MatrixXf mat = MatrixXf::Random(size, size);
  eigen_extensions::save(mat, "matxf.eig");
  MatrixXf mat2;
  eigen_extensions::load("matxf.eig", &mat2);
  EXPECT_TRUE(mat.isApprox(mat2));
}

TEST(EigenExtensions, VectorXd_serialization) {
  VectorXd mat = VectorXd::Random(size);
  eigen_extensions::save(mat, "vecxd.eig");
  VectorXd mat2;
  eigen_extensions::load("vecxd.eig", &mat2);
  EXPECT_TRUE(mat.isApprox(mat2));
  MatrixXd mat3;
  eigen_extensions::load("vecxd.eig", &mat3);
  EXPECT_TRUE(mat.isApprox(mat3));
}

TEST(EigenExtensions, VectorXf_serialization) {
  VectorXf mat = VectorXf::Random(size);
  eigen_extensions::save(mat, "vecxf.eig");
  VectorXf mat2;
  eigen_extensions::load("vecxf.eig", &mat2);
  EXPECT_TRUE(mat.isApprox(mat2));
  MatrixXf mat3;
  eigen_extensions::load("vecxf.eig", &mat3);
  EXPECT_TRUE(mat.isApprox(mat3));
}

TEST(EigenExtensions, Vector3i_serialization) {
  Vector3i mat = Vector3i::Random(3);
  eigen_extensions::save(mat, "vec3i.eig");
  Vector3i mat2;
  eigen_extensions::load("vec3i.eig", &mat2);
  EXPECT_TRUE(mat.isApprox(mat2));
  cout << mat.transpose() << endl;
  cout << mat2.transpose() << endl;
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
