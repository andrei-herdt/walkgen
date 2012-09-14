#pragma once
#ifndef MPC_WALKGEN_TYPES_H
#define MPC_WALKGEN_TYPES_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	types.h
///\brief	Definition of types used in MPC
///\author	Herdt Andrei
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/sharedtypes.h>//TODO: Include this?

#include <Eigen/Dense>

namespace MPCWalkgen{


  static const double EPSILON = 0.000001;


  //
  // Enums:
  //
  enum HullType{ FootHull, CoPHull };

  enum QPMatrixType{ matrixQ, matrixA };

  enum QPVectorType{ vectorP,  vectorBU,  vectorBL,  vectorXU,  vectorXL };

  enum Derivative { 
	  POSITION		= 0, 
	  VELOCITY		= 1, 
	  ACCELERATION	= 2, 
	  JERK			= 3, 
	  COP			= 4 };

  enum SampleRate { QP, ACTUATORS };


  //
  // Typedefs:
  //
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> EigenMatrixXdRM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> EigenMatrixXdCM;
  typedef EigenMatrixXdRM CommonMatrixType;

  typedef Eigen::VectorXd CommonVectorType;

  //
  // Data structures:
  //
  struct LinearDynamicsMatrices{
    CommonMatrixType S;
    CommonMatrixType U;
    CommonMatrixType UT;
    CommonMatrixType UInv;
    CommonMatrixType UInvT;
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
    CommonMatrixType V;
    CommonMatrixType VT;
    Eigen::VectorXd VcX;
    Eigen::VectorXd VcY;
    CommonMatrixType Vf;
    Eigen::VectorXd VcfX;
    Eigen::VectorXd VcfY;

    SelectionMatrices(const MPCData &mpc_parameters);
  };

  struct RelativeInequalities{
    CommonMatrixType DX;
    CommonMatrixType DY;
    Eigen::VectorXd Dc;

    void resize(int rows, int cols);
  };
}

#endif // MPC_WALKGEN_TYPES_H
