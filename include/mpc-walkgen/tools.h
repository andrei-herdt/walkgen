#pragma once
#ifndef MPC_WALKGEN_TOOLS_H
#define MPC_WALKGEN_TOOLS_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	tools.h
///\brief	A class of tools
///\author      Keith François
///\author	Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////

#include <Eigen/Dense>

#ifdef WIN32
#include <cmath>
inline double round( double d )
{	return floor( d + 0.5 );	}
#endif /* WIN32 */


namespace MPCWalkgen{
	void inverse(const Eigen::MatrixXd &X, Eigen::MatrixXd &Xinv, double eps=1e-8);

	inline double pow2(double v);

	inline double pow3(double v);

	inline double pow4(double v);

	inline double pow5(double v);


	/// \brief compute the value of the polynomial of degree 5 p(x)
	inline double p(const Eigen::Matrix<double,6,1> &factor, double x);

	/// \brief compute the value of the derivative of the polynomial of degree 5 p(x)
	inline double dp(const Eigen::Matrix<double,6,1> &factor, double x);

	/// \brief compute the value of the second derivative of the polynomial of degree 5 p(x)
	inline double ddp(const Eigen::Matrix<double,6,1> &factor, double x);


	// \brief Return true if the matrix given is triangular.
	// \param [in] m
	bool isUpperTriangular(const Eigen::MatrixXd &m);

	// \brief Return true if the matrix given is triangular.
	// \param [in] m
	bool hasCholeskyForm(const Eigen::MatrixXd &m);

	// \brief Return true if the matrix given is sparse such as
	//   every elmt is null except elemt for
	//    (i,i), (i, N+i), (N+i, i), (N+i, N+i), i in [0, N].
	//    where N is the half size of the matrix.
	// \param [in] R the rotation matrix block diagonal
	bool isSparseRotationMatrix (const Eigen::MatrixXd &rot);

	// \brief Return true if the matrix given is block diagonal
	// \param [in] R the rotation matrix block diagonal
	bool isDiagonalRotationMatrix(const Eigen::MatrixXd &rot);

	// \brief Compute the multiplication of the rotation matrix R by M
	// \param [in out] mInOut = R mInOut R^T
	// \param [in] R the rotation matrix block diagonal
	void computeRM(Eigen::MatrixXd &mInOut, const Eigen::MatrixXd &rot);


	// \brief Compute the multiplication of the rotation matrix M by the R^t
	// \param [in out] mInOut = R mInOut R^T
	// \param [in] R the rotation matrix block diagonal
	void computeMRt(Eigen::MatrixXd &mInOut, const Eigen::MatrixXd &rot);

	// \brief This method considers only blocks of matrices 2.2, so as to reduce the
	// number of multiplications.
	// \param [in out] mInOut = R mInOut R^T
	// \param [in] R the rotation matrix block diagonal
	void rotateCholeskyMatrix(Eigen::MatrixXd &mInOut, const Eigen::MatrixXd &rot);
}

#include <mpc-walkgen/tools-inl.h>

#endif // MPC_WALKGEN_TOOLS_H
