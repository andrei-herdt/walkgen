#include <mpc-walkgen/tools.h>

#include <Eigen/SVD>
#include <Eigen/LU>

#include <iostream>
#ifdef WIN32
# include <cmath>
#endif

using namespace Eigen;

void MPCWalkgen::inverse(const CommonMatrixType &A, CommonMatrixType &Ap, double eps) {
	FullPivLU<CommonMatrixType> lu(A);
	Ap = lu.inverse();
	for(int i = 0;i < Ap.rows(); ++i){
		for(int j = 0;j < Ap.cols(); ++j){
			if (fabs(Ap(i,j)) < eps){
				Ap(i,j) = 0;
			}
		}
	}

}

bool MPCWalkgen::isUpperTriangular(const CommonMatrixType &m)
{
	for (int i=0; i<m.rows(); ++i)
		for (int j=0; j<i; ++j)
			if (fabs(m(i,j)) > kEps)
				return false;

	return true;
}

void MPCWalkgen::rotateCholeskyMatrix(CommonMatrixType &mInOut, const CommonMatrixType &rot)
{
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

void MPCWalkgen::computeRM(CommonMatrixType &mIn, const CommonMatrixType &rot)
{
	// first step: compute rot*chol
	for (int i=0; i<mIn.rows()/2; ++i)
	{
		const Eigen::Matrix2d &rot_i = rot.block<2,2>(2*i, 2*i);
		for (int j=0; j<mIn.cols()/2; ++j)
			mIn.block<2,2>(2*i, 2*j) = rot_i*mIn.block<2,2>(2*i, 2*j);
	}
}

void MPCWalkgen::computeMRt(CommonMatrixType &mIn, const CommonMatrixType &rot)
{
	// compute chol*col^T
	for (int j=0; j<mIn.cols()/2; ++j) {
		Matrix2d rot_j = rot.block<2,2>(2*j, 2*j);
		rot_j.transposeInPlace();
		for (int i=0; i < mIn.rows() / 2; ++i)
			mIn.block<2,2>(2*i, 2*j) = mIn.block<2,2>(2*i, 2*j) * rot_j;
	}


}
