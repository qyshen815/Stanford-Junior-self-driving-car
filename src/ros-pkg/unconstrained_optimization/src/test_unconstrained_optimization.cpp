
#include <unconstrained_optimization/unconstrained_optimization.h>
#include <unconstrained_optimization/common_functions.h>
#include <gtest/gtest.h>

using namespace std;
using namespace Eigen;    

void generateRandomProblemData(MatrixXd* A, VectorXd* b,
			       Eigen::SparseMatrix<double, Eigen::ColMajor>* A_sparse)
{
  // -- Problem setup.
  int num_vars = 10;
  int num_tr_ex = 1000;
  *b = VectorXd::Random(num_tr_ex);
  
  // -- Set up dense version of A.
  *A = MatrixXd::Zero(num_vars, num_tr_ex);
  int num_nonzero = 0;
  for(int i = 0; i < A->rows(); ++i) {
    for(int j = 0; j < A->cols(); ++j) {
      double val = (double)rand() / (double)RAND_MAX;
      if(val > 0.9) {
	A->coeffRef(i, j) = val;
	++num_nonzero;
      }
    }
  }
  
  // -- Set up sparse version of A.
  *A_sparse = Eigen::SparseMatrix<double, Eigen::ColMajor>(A->rows(), A->cols());
//   cout << "A: " << A.rows() << " x " << A.cols() << endl;
//   cout << "A_sparse: " << A_sparse.rows() << " x " << A_sparse.cols() << endl;

  A_sparse->startFill(num_nonzero * 1.1);
  int filled = 0;
  for(int j = 0; j < A->cols(); ++j) {
    for(int i = 0; i < A->rows(); ++i) {
      if(A->coeff(i, j) != 0) { 
	A_sparse->fill(i, j) = A->coeff(i, j);
	++filled;
      }
    }
  }
  A_sparse->endFill();
  assert(filled == num_nonzero);
}

TEST(SparseMLS, correctness)
{

//   cout << "A: " << endl << A << endl;
//   cout << "A_sparse: " << endl << A_sparse << endl;
  MatrixXd A;
  VectorXd b;
  Eigen::SparseMatrix<double, Eigen::ColMajor> A_sparse;
  generateRandomProblemData(&A, &b, &A_sparse);

  // -- Get mean logistic score for a random vector using both methods.
  VectorXd x = VectorXd::Random(A.rows());
  ObjectiveMLS obj_mls(A, b);
  double regular = obj_mls.eval(x);

  ObjectiveMLSSparse obj_mls_sparse(&A_sparse, &b);
  double sparse = obj_mls_sparse.eval(x);

  cout << "Delta: " << regular - sparse << endl;
  EXPECT_DOUBLE_EQ(regular, sparse);

  // -- Get gradient with both methods.
  GradientMLS grad_mls(A, b);
  VectorXd grad = grad_mls.eval(x);
  GradientMLSSparse grad_mls_sparse(&A_sparse, &b);
  VectorXd grad_sparse = grad_mls_sparse.eval(x);
  cout << grad.transpose() << endl << grad_sparse.transpose() << endl;
  cout << "Norm of the gradient difference: " << (grad - grad_sparse).norm() << endl;
  for(int i = 0; i < grad.rows(); ++i)
    EXPECT_DOUBLE_EQ(grad(i), grad_sparse(i));

  // -- Get Hessian with both methods.
  HessianMLS hess_mls(A, b);
  MatrixXd hess = hess_mls.eval(x);
  HessianMLSSparse hess_mls_sparse(&A_sparse, &b);
  MatrixXd hess_sparse = hess_mls_sparse.eval(x);
  double fn = 0;
  for(int i = 0; i < hess.rows(); ++i) { 
    for(int j = 0; j < hess.cols(); ++j) { 
      EXPECT_DOUBLE_EQ(hess(i, j), hess_sparse(i, j));
      fn += pow(hess(i, j) - hess_sparse(i, j), 2);
    }
  }
  cout << "Frobenius norm of hessian difference: " << sqrt(fn) << endl;
//   cout << hess << endl;
//   cout << hess_sparse << endl;
    
}

TEST(UnconstrainedOptimization, SparseMEL_matches_MEL)
{
  MatrixXd A;
  VectorXd b;
  Eigen::SparseMatrix<double, Eigen::ColMajor> A_sparse;
  generateRandomProblemData(&A, &b, &A_sparse);

  for(int j = 0; j < 10; ++j) { 
    // -- Get mean exponential loss for a random vector using both methods.
    VectorXd x = VectorXd::Random(A.rows());
    ObjectiveMEL obj_mel(A, b);
    double regular = obj_mel.eval(x);
    
    ObjectiveMELSparse obj_mel_sparse(&A_sparse, &b);
    double sparse = obj_mel_sparse.eval(x);
    
    cout << "Delta: " << regular - sparse << endl;
    EXPECT_FLOAT_EQ(regular, sparse);

    // -- Get gradient with both methods.
    GradientMEL grad_mel(A, b);
    VectorXd grad = grad_mel.eval(x);
    GradientMELSparse grad_mel_sparse(&A_sparse, &b);
    VectorXd grad_sparse = grad_mel_sparse.eval(x);
    cout << grad.transpose() << endl << grad_sparse.transpose() << endl;
    cout << "Norm of the gradient difference: " << (grad - grad_sparse).norm() << endl;
    for(int i = 0; i < grad.rows(); ++i)
      EXPECT_FLOAT_EQ(grad(i), grad_sparse(i));
  }
  
//   // -- Get Hessian with both methods.
//   HessianMEL hess_mel(A, b);
//   MatrixXd hess = hess_mel.eval(x);
//   HessianMELSparse hess_mel_sparse(&A_sparse, &b);
//   MatrixXd hess_sparse = hess_mel_sparse.eval(x);
//   double fn = 0;
//   for(int i = 0; i < hess.rows(); ++i) { 
//     for(int j = 0; j < hess.cols(); ++j) { 
//       EXPECT_DOUBLE_EQ(hess(i, j), hess_sparse(i, j));
//       fn += pow(hess(i, j) - hess_sparse(i, j), 2);
//     }
//   }
//   cout << "Frobenius norm of hessian difference: " << sqrt(fn) << endl;

  
}

TEST(UnconstrainedOptimization, NewtonSolver)
{
  int num_vars = 10;
  int num_tr_ex = 1000;
  MatrixXd A = MatrixXd::Random(num_vars, num_tr_ex);
  VectorXd b = VectorXd::Random(num_tr_ex);

  ObjectiveMLS obj_mls(A, b);
  GradientMLS grad_mls(A, b);
  HessianMLS hess_mls(A, b);

  cout << "Solving." << endl;
  VectorXd init = VectorXd::Zero(A.rows());
  double tol = 1e-6;
  double alpha = 0.3;
  double beta = 0.5;
  double stepsize = 1.0;
  NewtonSolver solver_mls(&obj_mls, &grad_mls, &hess_mls, tol, alpha, beta, stepsize, true);
  VectorXd solution = solver_mls.solve(init);
  cout << "Got solution:" << endl << solution.transpose() << endl;

  cout << "Perturbing..." << endl;
  for(int i = 0; i < 1e4; ++i) {
    VectorXd random = VectorXd::Random(solution.rows());
    random *= ((double)rand() / (double)RAND_MAX);
    VectorXd perturbed = solution + random;

    EXPECT_TRUE(obj_mls.eval(perturbed) + 1e-6 >= obj_mls.eval(solution));
  }

  cout << "Solving with Nesterov." << endl;
  NesterovGradientSolver ngs(&obj_mls, &grad_mls, tol, alpha, beta, 0, 1, true);
  ngs.hessian_ = &hess_mls; // Compute condition number.
  VectorXd nesterov_solution = ngs.solve(init);
  cout << "Got solution:" << endl << nesterov_solution.transpose() << endl;
  
  cout << "Perturbing..." << endl;
  for(int i = 0; i < 1e4; ++i) {
    VectorXd random = VectorXd::Random(nesterov_solution.rows());
    random *= ((double)rand() / (double)RAND_MAX);
    VectorXd perturbed = nesterov_solution + random;

    EXPECT_TRUE(obj_mls.eval(perturbed) + 1e-6 >= obj_mls.eval(solution));
  }


  cout << "Solving with Gradient." << endl;
  GradientSolver gs(&obj_mls, &grad_mls, tol, alpha, beta, 0, 1, true);
  VectorXd gradient_solution = gs.solve(init);
  cout << "Got solution:" << endl << gradient_solution.transpose() << endl;

  cout << "Perturbing..." << endl;
  double gradient_objective = obj_mls.eval(gradient_solution);
  for(int i = 0; i < 1e4; ++i) {
    VectorXd random = VectorXd::Random(gradient_solution.rows());
    random *= ((double)rand() / (double)RAND_MAX);
    VectorXd perturbed = gradient_solution + random;
    EXPECT_TRUE(obj_mls.eval(perturbed) >= gradient_objective);
  }
}


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
