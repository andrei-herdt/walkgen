#pragma once
#ifndef MPC_WALKGEN_INTERPOLATION_H
#define MPC_WALKGEN_INTERPOLATION_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	interpolation.h
///\author	Andrei Herdt
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

    void computeInterpolationByJerk(Eigen::VectorXd &solutionX, Eigen::VectorXd &solutionY, const BodyState &state,
      const LinearDynamicsMatrices &dyn, double jerkX, double jerkY) const;

    void computeInterpolationByJerk(Eigen::VectorXd &solution, const Eigen::VectorXd &state,
      const LinearDynamicsMatrices &dyn, double jerk) const;

    void Interpolate(Eigen::VectorXd &solution_vec, const LinearDynamicsMatrices &dyn, 
      const Eigen::Vector3d &state_vec, const Eigen::VectorXd &u_vec);

    void Interpolate(Eigen::VectorXd &solution_vec, const LinearDynamicsMatrices &dyn, 
      const Eigen::Vector3d &state_vec, double u);

    void computePolynomialNormalisedFactors(
      Eigen::Matrix<double,6,1> &factor, const Eigen::Vector3d &initialstate,
      const Eigen::Vector3d &finalState, double T ) const;

    void computePolynomialFactors(
      Eigen::Matrix<double,6,1>  &factor,
      const Eigen::Vector3d &initialstate,
      const Eigen::Vector3d &finalState, double T ) const;

  private:
    Eigen::Matrix3d AinvNorm_;
    Eigen::VectorXd tmp_vec_;
  };
}

#endif // MPC_WALKGEN_INTERPOLATION_H
