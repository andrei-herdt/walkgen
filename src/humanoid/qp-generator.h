#pragma once
#ifndef MPC_WALKGEN_QP_GENERATOR_H
#define MPC_WALKGEN_QP_GENERATOR_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	qp-generator.h
///\brief	A class to compute QP elements (objective, constraints, warmstart)
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author      Keith François
///\version	1.2
///\date	27/04/12
///
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
        RigidBodySystem * robot, const MPCData * generalData);
      ~QPGenerator();

      void precomputeObjective();

      void BuildProblem(MPCSolution & solution);

      void computeWarmStart(MPCSolution & solution);

      void ConvertCopToJerk(MPCSolution & solution);

      void computeReferenceVector(const MPCSolution & solution);

      void display(const MPCSolution & solution, const std::string & filename) const;

    private:

      void buildInequalitiesFeet(const MPCSolution & solution);

      void buildObjective(const MPCSolution & solution);

      void buildConstraints(const MPCSolution & solution);

      void buildConstraintsFeet(const MPCSolution & solution);

      void buildConstraintsCOP(const MPCSolution & solution);

    private:

      QPPreview * preview_;
      QPSolver * solver_;
      RigidBodySystem * robot_;
      Reference * velRef_;
      QPPonderation * ponderation_;
      const MPCData * generalData_;

      Eigen::VectorXd tmpVec_;
      Eigen::VectorXd tmpVec2_;
      Eigen::MatrixXd tmpMat_;
      Eigen::MatrixXd tmpMat2_;

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

/*! \fn MPCWalkgen::QPGenerator::QPGenerator(QPPreview * preview, QPSolver * solver,
*					VelReference * velRef, QPPonderation * ponderation,
*					RigidBodySystem * robot, const MPCData * generalData)
* \brief Constructor
*/

/*! \fn MPCWalkgen::QPGenerator::buildObjective(const MPCSolution & solution)
* \brief Build matrix Q and vector p for the QP
*/

/*! \fn MPCWalkgen::QPGenerator::buildConstraints(const MPCSolution & solution)
* \brief build matrix A and vectors BU, BL, XU, XL for the QP
*/

/*! \fn MPCWalkgen::QPGenerator::computeWarmStart(MPCSolution & solution)
* \brief Compute a feasible solution and an active-set of previewed constraints
*/

/*! \fn MPCWalkgen::QPGenerator::convertCopToJerk(MPCSolution & solution)
* \brief the current QP generate a solution in CoP position. for the interpolation, we must to convert the solution into CoM Jerk
*/

/*! \fn MPCWalkgen::QPGenerator::computeReferenceVector(const MPCSolution & solution)
* \brief Compute the reference vector (constant over the horizon)
*/

/*! \fn MPCWalkgen::QPGenerator::display(const MPCSolution & solution, const std::string & filename) const
* \brief display some informations of the solution in a file (previewed CoM, CoP and feet and CoP constraints)
* \param solution   the solution
* \param filename the file to display solution
*/

#endif // MPC_WALKGEN_QP_GENERATOR_H
