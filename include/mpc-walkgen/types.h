#pragma once
#ifndef MPC_WALKGEN_TYPES_H
#define MPC_WALKGEN_TYPES_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	types.h
///\brief	Definition of types used in MPC
///\author	Herdt Andrei
////////////////////////////////////////////////////////////////////////////////


#include <Eigen/Dense>

#include <mpc-walkgen/sharedpgtypes.h>//TODO: Why is this included?

namespace MPCWalkgen{

  enum HullType{
    FootHull,
    CoPHull
  };

  static const double EPSILON = 0.000001;

  enum QPMatrixType{
    matrixQ,
    matrixA
  };

  enum QPVectorType{
    vectorP,
    vectorBU,
    vectorBL,
    vectorXU,
    vectorXL
  };

  enum Derivative {
    POSITION,
    VELOCITY,
    ACCELERATION,
    JERK,
    COP
  };

  enum SampleRate {
    QP,
    ACTUATORS
  };

  struct LinearDynamicsMatrices{
    Eigen::MatrixXd S;
    Eigen::MatrixXd U;
    Eigen::MatrixXd UT;
    Eigen::MatrixXd UInv;
    Eigen::MatrixXd UInvT;
  };

  struct LinearDynamics {
    LinearDynamicsMatrices pos, vel, acc, jerk, cop;
  };

  struct Reference{
    struct Frame{
      Eigen::VectorXd x;
      Eigen::VectorXd y;
      Eigen::VectorXd yaw;

      Frame();

      void resize(int size);
    };

    Frame global;
    Frame local;

    Reference();

    void resize(int size);
  };

  struct SelectionMatrices{
    Eigen::MatrixXd V;
    Eigen::MatrixXd VT;
    Eigen::VectorXd VcX;
    Eigen::VectorXd VcY;
    Eigen::MatrixXd Vf;
    Eigen::VectorXd VcfX;
    Eigen::VectorXd VcfY;

    SelectionMatrices(const MPCData &data_mpc);
  };


  struct RelativeInequalities{
    Eigen::MatrixXd DX;
    Eigen::MatrixXd DY;
    Eigen::VectorXd Dc;

    void resize(int rows, int cols);
  };
}

/** @defgroup private MPCWalkgen private interface
*  This group gathers the classes contained in the private interface
*/


#endif // MPC_WALKGEN_TYPES_H
