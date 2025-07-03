#include <iostream>
#include <Eigen/Dense>
#define N_BL 6
#define N_TEL 4

Eigen::Matrix<double, N_TEL, N_TEL> singularDiag = Eigen::Matrix<double, N_TEL, N_TEL>::Zero();

const Eigen::Matrix<double, N_TEL, N_BL> M_lacour_dag = (Eigen::Matrix<double, N_TEL, N_BL>() << 
    -0.25, -0.25, -0.25, 0, 0, 0,
    0.25, 0, 0, -0.25, -0.25, 0,
    0, 0.25, 0, 0.25, 0, -0.25,
    0, 0, 0.25, 0, 0.25, 0.25).finished();

const Eigen::Matrix<double, N_BL, N_TEL> M_lacour = (Eigen::Matrix<double, N_BL, N_TEL>() << 
    -1, 1, 0, 0,
    -1, 0, 1, 0,
    -1, 0, 0, 1,
    0, -1, 1, 0,
    0, -1, 0, 1,
    0, 0, -1, 1).finished();


#define NUMERIC_LIMIT 2e-6
Eigen::Matrix4d make_pinv(Eigen::Matrix<double, N_BL, N_BL> W, double threshold){
    using namespace Eigen;
    // This function computes the pseudo-inverse of the matrix M^T *  W * M, using the
    // SVD method. The threshold is used to set the minimum eigenvalue, and the
    // minimum eigenvalue is used to set the minimum eigenvalue of the pseudo-inverse.
    // W * M_lacour is a 6x4 matrix, and M_lacour.transpose() * W * M_lacour is a 4x4 matrix.
    // Was ComputeThinU
    //JacobiSVD<Matrix4d, ComputeFullU | ComputeFullV> svd(M_lacour.transpose() * W * M_lacour);
    SelfAdjointEigenSolver<Matrix4d> es(M_lacour.transpose() * W * M_lacour);
    // Start with a diagonal vector of 4 zeros.
    for (int i=0; i<N_TEL; i++){
         if ((es.eigenvalues()(i) <= threshold) || (es.eigenvalues()(i) < NUMERIC_LIMIT)){
             if (threshold > 0){
                 singularDiag(i,i) = es.eigenvalues()(i)/threshold/threshold;
            } else singularDiag(i,i) = 0;
        } else {
            singularDiag(i,i) = 1.0/es.eigenvalues()(i);
        }
    }
    std::cout << "first eigenvalue = " << es.eigenvalues()(0) << std::endl;
    std::cout << "eigenvalues = " << es.eigenvalues().transpose() << std::endl;
    return  es.eigenvectors() * singularDiag * es.eigenvectors().transpose();
}

int main()
{ 
  Eigen::Matrix<double, N_BL, 1> X;
  Eigen::Vector4d Y;
  X(0)=1000;
  X(1)=1000;
  X(2)=1000;
  X(3)=1000;
  X(4)=0;
  X(5)=0;
  std::cout << "m =" << std::endl << M_lacour_dag << std::endl;
  std::cout << "m * X =" << std::endl << M_lacour_dag * X << std::endl;

  Eigen::DiagonalMatrix<double, N_BL> W(X);
  Eigen::Matrix4d pinv = make_pinv(W, 0);
  std::cout << "pinv =" << std::endl << pinv << std::endl;
  std::cout << "I6GD =" << std::endl << M_lacour * pinv * M_lacour.transpose() * W << std::endl;
  return 0;
}
