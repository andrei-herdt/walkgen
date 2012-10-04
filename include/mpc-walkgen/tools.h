#pragma once
#ifndef MPC_WALKGEN_TOOLS_H
#define MPC_WALKGEN_TOOLS_H

#include <mpc-walkgen/types.h>

#include <Eigen/Dense>

#ifdef WIN32
#include <cmath>
#endif /* WIN32 */

namespace MPCWalkgen{
void inverse(const CommonMatrixType &X, CommonMatrixType &Xinv);

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
bool isUpperTriangular(const CommonMatrixType &m);

// \brief Return true if the matrix given is triangular.
// \param [in] m
bool hasCholeskyForm(const CommonMatrixType &m);

// \brief Compute the multiplication of the rotation matrix R by M
// \param [in out] mInOut = R mInOut R^T
// \param [in] R the rotation matrix block diagonal
void ComputeRM(CommonMatrixType &mInOut,
		const CommonMatrixType &rot
		);


// \brief Compute the multiplication of the rotation matrix M by the R^t
// \param [in out] mInOut = R mInOut R^T
// \param [in] R the rotation matrix block diagonal
void computeMRt(CommonMatrixType &mInOut,
		const CommonMatrixType &rot
);

// \brief This method considers only blocks of matrices 2.2, so as to reduce the
// number of multiplications.
// \param [in out] mInOut = R mInOut R^T
// \param [in] R the rotation matrix block diagonal
void RotateCholeskyMatrix(CommonMatrixType &mInOut,
		const CommonMatrixType &rot
		);
}

#include <mpc-walkgen/tools-inl.h>

#endif // MPC_WALKGEN_TOOLS_H
