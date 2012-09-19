#pragma once
#ifndef MPC_WALKGEN_QP_GENERATOR_H
#define MPC_WALKGEN_QP_GENERATOR_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	qp-generator.h
///\author	Herdt Andrei
///\author Jory Lafaye
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/types.h>
#include <mpc-walkgen/qp-solver.h>
#include <mpc-walkgen/qp-preview.h>

#include <Eigen/Dense>
#include <vector>

namespace MPCWalkgen{
  class QPGenerator{

    //
    // Public methods:
    //
  public:
    QPGenerator(QPPreview * preview, QPSolver * solver,
      Reference * ref, WeightCoefficients * weight_coefficients,
      RigidBodySystem * robot, const MPCData * mpc_parameters);
    ~QPGenerator();

    void precomputeObjective();

    void BuildProblem(MPCSolution &solution);

    void computeWarmStart(MPCSolution &solution);

    void ConvertCopToJerk(MPCSolution &solution);

    void computeReferenceVector(const MPCSolution &solution);

  private:

    void buildInequalitiesFeet(const MPCSolution &solution);

    void buildObjective(const MPCSolution &solution);

    void buildConstraints(const MPCSolution &solution);

    void buildConstraintsFeet(const MPCSolution &solution);

    void buildConstraintsCOP(const MPCSolution &solution);

  private:

    QPPreview *preview_;
    QPSolver *solver_;
    RigidBodySystem *robot_;
    Reference *ref_;
    WeightCoefficients *weight_coefficients_;
    const MPCData *mpc_parameters_;

    Eigen::VectorXd tmp_vec_;
    Eigen::VectorXd tmp_vec2_;
    CommonMatrixType tmp_mat_;
    CommonMatrixType tmp_mat2_;

    RelativeInequalities feetInequalities_;//TODO: Maybe should be instantiated in robot_

    std::vector<CommonMatrixType> Qconst_;
    std::vector<CommonMatrixType> QconstN_;
    std::vector<CommonMatrixType> choleskyConst_;
    std::vector<CommonMatrixType> pconstCoM_;
    std::vector<CommonMatrixType> pconstVc_;
    std::vector<CommonMatrixType> pconstRef_;

    ConvexHull FootFeasibilityEdges;
    ConvexHull COPFeasibilityEdges;
    ConvexHull hull;
  };
}
#endif // MPC_WALKGEN_QP_GENERATOR_H
