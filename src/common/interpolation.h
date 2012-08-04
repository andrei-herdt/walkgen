#pragma once
#ifndef MPC_WALKGEN_INTERPOLATION_H
#define MPC_WALKGEN_INTERPOLATION_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	interpolation.h
///\brief	A tools class wich provide some interpolation methods
///\author	Andrei Herdt
///\author	Lafaye Jory
///\author      Keith François
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////



#include "types.h"
#include "tools.h"
#include <Eigen/Dense>

namespace MPCWalkgen{

	class Interpolation{

		public:
				Interpolation();
				~Interpolation();

				void computeInterpolationByJerk(Eigen::VectorXd & solutionX, Eigen::VectorXd & solutionY, const BodyState & state,
						 const LinearDynamics & dyn, double jerkX, double jerkY) const;

				void computeInterpolationByJerk(Eigen::VectorXd & solution, const Eigen::VectorXd & state,
						 const LinearDynamics & dyn, double jerk) const;

        Eigen::VectorXd Interpolate(const LinearDynamics &dyn, 
          const Eigen::Vector3d &state_vec, const Eigen::VectorXd &u_vec) const;

				void computePolynomialNormalisedFactors(
						Eigen::Matrix<double,6,1> & factor, const Eigen::Vector3d & initialstate,
						const Eigen::Vector3d & finalState, double T ) const;

				void computePolynomialFactors(
						Eigen::Matrix<double,6,1>  & factor,
						const Eigen::Vector3d & initialstate,
						const Eigen::Vector3d & finalState, double T ) const;
		private:
				Eigen::Matrix3d AinvNorm_;
	};
}

/*! \fn MPCWalkgen::Interpolation::Interpolation()
* \brief Constructor
*/


#endif // MPC_WALKGEN_INTERPOLATION_H
