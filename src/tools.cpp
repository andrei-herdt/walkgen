#include <mpc-walkgen/tools.h>

#include <Eigen/SVD>
#include <Eigen/LU>

#include <iostream>
#ifdef WIN32
# include <cmath>
#endif

using namespace Eigen;

void MPCWalkgen::Invert(const CommonMatrixType &A, CommonMatrixType &Ap) {
	FullPivLU<CommonMatrixType> lu(A);
	Ap = A.inverse();//lu.inverse();
	for(int i = 0;i < Ap.rows(); ++i){//TODO: Remove this
		for(int j = 0;j < Ap.cols(); ++j){
			if (fabs(Ap(i,j)) < kEps){
				Ap(i,j) = 0;
			}
		}
	}

}

void MPCWalkgen::RotateCholeskyMatrix(CommonMatrixType &mInOut, const CommonMatrixType &rot) {
	int N = mInOut.rows();
	int n2 = N/2;

	for (int j=0; j<n2; ++j){
		Eigen::Matrix2d rotT_j= rot.block<2,2>(2*j, 2*j);
		rotT_j.transposeInPlace();
		for (int i=0; i<j; ++i){
			mInOut.block<2,2>(2*i, 2*j) =
					rot.block<2,2>(2*i, 2*i) * mInOut.block<2,2>(2*i, 2*j)* rotT_j;
		}
	}
}

void MPCWalkgen::RTimesM(CommonMatrixType &m, const CommonMatrixType &r) {
	// first step: compute rot*chol
	for (int i=0; i<m.rows()/2; ++i)
	{
		const Eigen::Matrix2d &rot_i = r.block<2,2>(2*i, 2*i);
		for (int j=0; j<m.cols()/2; ++j)
			m.block<2,2>(2*i, 2*j) = rot_i*m.block<2,2>(2*i, 2*j);
	}
}

void MPCWalkgen::MTimesRT(CommonMatrixType &m, const CommonMatrixType &rt) {
	// compute chol*col^T
	for (int j=0; j<m.cols()/2; ++j) {
		Matrix2d rot_j = rt.block<2,2>(2*j, 2*j);
		rot_j.transposeInPlace();
		for (int i=0; i < m.rows() / 2; ++i)
			m.block<2,2>(2*i, 2*j) = m.block<2,2>(2*i, 2*j) * rot_j;
	}
}
