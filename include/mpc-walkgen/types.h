#pragma once
#ifndef MPC_WALKGEN_TYPES_H
#define MPC_WALKGEN_TYPES_H

#include <mpc-walkgen/sharedtypes.h>

namespace MPCWalkgen{

  //
  // Enums: 
  //
  enum HullType{ FOOT_HULL, COP_HULL };//TODO: Remove this

  enum QPMatrixType{ HESSIAN, matrixA };//TODO: Remove this

  enum QPVectorType{ vectorP,  vectorBU,  vectorBL,  vectorXU,  vectorXL };//TODO: Remove this

  enum Derivative { 
	  POSITION		= 0, 
	  VELOCITY		= 1, 
	  ACCELERATION	= 2, 
	  JERK			= 3, 
	  COP			= 4
  };

  enum SampleRate { QP, ACTUATORS };//TODO: Needed?

  //
  // Data structures:
  //
  struct LinearDynamicsMatrices{
    CommonMatrixType state_mat;
    CommonMatrixType input_mat;
    CommonMatrixType input_mat_tr;
    CommonMatrixType input_mat_inv;
    CommonMatrixType input_mat_inv_tr;

    void SetZero(double state_dimension, double num_samples);
  };

  struct LinearDynamics {
    LinearDynamicsMatrices pos, vel, acc, jerk, cop;

    void SetZero(double state_dimension, double input_dimension);
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

  struct RelativeInequalities{//TODO: Obsolete
    CommonMatrixType DX;
    CommonMatrixType DY;
    CommonVectorType Dc;

    void resize(int rows, int cols);
  };
}

#endif // MPC_WALKGEN_TYPES_H
