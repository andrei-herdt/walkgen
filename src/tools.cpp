#include <mpc-walkgen/tools.h>

#include <Eigen/SVD>
#include <Eigen/LU>

#include <iostream>
#ifdef WIN32
# include <cmath>
#endif

using namespace Eigen;

void MPCWalkgen::inverse(const MatrixXd &A, MatrixXd &Ap, double eps) {
	FullPivLU<MatrixXd> lu(A);
	Ap = lu.inverse();
	for(int i=0;i<Ap.rows();++i){
		for(int j=0;j<Ap.cols();++j){
			if (fabs(Ap(i,j))<eps){
				Ap(i,j)=0;
			}
		}
	}

}

// Every elmt is supposed to be 0, execpted the diagonal.
bool MPCWalkgen::isSparseRotationMatrix (const MatrixXd &rot1)
{
	int N = rot1.rows() / 2;//TODO: Safe?
	bool solution = true;
	for (int i = 0; i < 2 * N; ++i)
		for (int j = 0; j < 2 * N; ++j)
			solution = solution && ( fabs(rot1(i,j)) < kEps
					|| (i == j) || (i == j+N) || (i + N == j) );
	return solution;
}

//
bool MPCWalkgen::isDiagonalRotationMatrix(const Eigen::MatrixXd &rot)
{
	for (int i=0; i<rot.rows(); ++i)
	{
		for (int j=0; j<rot.cols(); ++j)
		{
			// the only non null elements should be on the diagonal.
			if (fabs(rot(i,j)) > kEps)
			{
				int halfi = 2* static_cast <int>(floor (i/2.) );
				int halfj = 2* static_cast <int>(floor (j/2.) );
				if(	halfi != halfj)
					return false;
			}
		}
	}
	return true;
}

bool MPCWalkgen::isUpperTriangular(const Eigen::MatrixXd &m)
{
	for (int i=0; i<m.rows(); ++i)
		for (int j=0; j<i; ++j)
			if (fabs(m(i,j)) > kEps)
				return false;

	return true;
}

bool MPCWalkgen::hasCholeskyForm(const Eigen::MatrixXd &m)
{
	//Check that the matrix is upper triangular,  (a)
	//such as m(2*i,2*i) = m(2*i+1,2*i+1)         (b)
	//and  as m(2*i+i,2*j) = 0                    (c)
	//and  as m(2*i  ,2*j+i) = 0                  (d)

	// (a)
	for (int i=0; i<m.rows(); ++i)
		for (int j=0; j<i; ++j)
			if (fabs(m(i,j)) > kEps)
				return false;


	int n2 = static_cast <int> (floor(m.rows()/2.));

	// (b)
	for (int i=0; i<n2; ++i)
		if( fabs(m(2*i+1,2*i+1) - m(2*i,2*i)) < kEps)
			return false;

	// (c)
	for (int i=0; i<n2; ++i)
		for (int j=0; j<n2; ++j)
			if ( fabs(m(2*i+1,2*j)) > kEps)
				return false;

	// (d)
	for (int i=0; i<n2; ++i)
		for (int j=0; j<n2; ++j)
			if( fabs(m(2*i,2*j+1)) > kEps)
				return false;

	return true;
}

void MPCWalkgen::rotateCholeskyMatrix(MatrixXd &mInOut, const MatrixXd &rot)
{
	assert(isDiagonalRotationMatrix(rot) && "The matrix rot is not 2.2 block diagonal");
	assert(hasCholeskyForm(mInOut) && "The cholesky matrix has not the form required for a cholesky matrix");

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
	assert(isUpperTriangular(mInOut) && "The cholesky matrix is not upper triangular at the exit of the function");
}

void MPCWalkgen::computeRM(MatrixXd &mIn, const MatrixXd &rot)
{
	assert(isDiagonalRotationMatrix(rot) && "The matrix rot is not 2.2 block diagonal");

	// first step: compute rot*chol
	for (int i=0; i<mIn.rows()/2; ++i)
	{
		const Eigen::Matrix2d &rot_i = rot.block<2,2>(2*i, 2*i);
		for (int j=0; j<mIn.cols()/2; ++j)
			mIn.block<2,2>(2*i, 2*j) = rot_i*mIn.block<2,2>(2*i, 2*j);
	}
}

void MPCWalkgen::computeMRt(MatrixXd &mIn, const MatrixXd &rot)
{
	assert(isDiagonalRotationMatrix(rot) && "The matrix rot is not 2.2 block diagonal");

	// compute chol*col^T
	for (int j=0; j<mIn.cols()/2; ++j)
	{
		Matrix2d rot_j = (rot.block<2,2>(2*j, 2*j));
		rot_j.transposeInPlace();
		for (int i=0; i<mIn.rows()/2; ++i)
			mIn.block<2,2>(2*i, 2*j) = mIn.block<2,2>(2*i, 2*j)*rot_j;
	}


}
