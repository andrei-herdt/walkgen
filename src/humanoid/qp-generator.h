#pragma once
#ifndef MPC_WALKGEN_QP_GENERATOR_H
#define MPC_WALKGEN_QP_GENERATOR_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	qp-generator.h
///\author	Herdt Andrei
////////////////////////////////////////////////////////////////////////////////



#include "types.h"

#include "../common/qp-solver.h"
#include "qp-preview.h"

#include <Eigen/Dense>
#include <vector>

namespace MPCWalkgen{
    class QPGenerator{

    public:
      QPGenerator(QPPreview * preview, QPSolver * solver,
        Reference * velRef, QPPonderation * ponderation,
        RigidBodySystem * robot, const MPCData * data_mpc);
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

      QPPreview * preview_;
      QPSolver * solver_;
      RigidBodySystem * robot_;
      Reference * velRef_;
      QPPonderation * ponderation_;
      const MPCData * data_mpc_;

      Eigen::VectorXd tmp_vec_;
      Eigen::VectorXd tmp_vec2_;
      Eigen::MatrixXd tmp_mat_;
      Eigen::MatrixXd tmp_mat2_;

      RelativeInequalities feetInequalities_;//TODO: Maybe should be instantiated in robot_

      std::vector<Eigen::MatrixXd> Qconst_;
      std::vector<Eigen::MatrixXd> QconstN_;
      std::vector<Eigen::MatrixXd> choleskyConst_;
      std::vector<Eigen::MatrixXd> pconstCoM_;
      std::vector<Eigen::MatrixXd> pconstVc_;
      std::vector<Eigen::MatrixXd> pconstRef_;

      ConvexHull FootFeasibilityEdges;
      ConvexHull COPFeasibilityEdges;
      ConvexHull hull;
    };
}
#endif // MPC_WALKGEN_QP_GENERATOR_H
