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

namespace MPCWalkgen{


  static const double EPSILON = 0.000001;

  //
  // Enums: 
  //
  enum HullType{ FootHull, CoPHull };//TODO: Remove this

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

  struct SelectionMatrices{
    CommonMatrixType sample_step, sample_step_trans;
    CommonVectorType sample_step_cx, sample_step_cy;
    CommonMatrixType Vf;
    CommonVectorType VcfX, VcfY;              			//\brief
    CommonVectorType sample_mstep_cx, sample_mstep_cy;  //\brief Middle of both currently supporting feet
    CommonMatrixType sample_mstep, sample_mstep_trans;	//\brief Middle of previewed feet

    void SetZero();
    SelectionMatrices(int num_rows);
  };

  struct RelativeInequalities{
    CommonMatrixType DX;
    CommonMatrixType DY;
    CommonVectorType Dc;

    void resize(int rows, int cols);
  };
}

#endif // MPC_WALKGEN_TYPES_H
