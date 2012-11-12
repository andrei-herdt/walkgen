#pragma once
#ifndef MPC_WALKGEN_INTERPOLATION_H
#define MPC_WALKGEN_INTERPOLATION_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	interpolation.h
///\author	Andrei Herdt
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/types.h>
#include <mpc-walkgen/tools.h>

#include <Eigen/Dense>

namespace MPCWalkgen{

  class Interpolation{

  public:
    Interpolation();
    ~Interpolation();

    void Interpolate(CommonVectorType &solution_vec, const LinearDynamicsMatrices &dyn, 
      const CommonVectorType &state_vec, const CommonVectorType &u_vec);

    void Interpolate(CommonVectorType &solution_vec, const LinearDynamicsMatrices &dyn, 
      const CommonVectorType &state_vec, double u);

    void Interpolate(CommonVectorType &solution_vec, const LinearDynamicsMatrices &dyn,
          const CommonVectorType &state_vec, double u, double mu);

    void ComputeNormalPolynomCoefficients(
      Eigen::Matrix<double,6,1> &factor, const Eigen::Vector3d &initialstate,
      const Eigen::Vector3d &finalState, double T ) const;

    void ComputePolynomCoefficients(
      Eigen::Matrix<double,6,1>  &factor,
      const Eigen::Vector3d &initialstate,
      const Eigen::Vector3d &finalState, double T ) const;

  private:
    Eigen::Matrix3d AinvNorm_;
    CommonVectorType tmp_vec_;
  };
}

#endif // MPC_WALKGEN_INTERPOLATION_H
