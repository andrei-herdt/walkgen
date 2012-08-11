#pragma once
#ifndef MPC_WALKGEN_COMMON_TYPE_H
#define MPC_WALKGEN_COMMON_TYPE_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	common-types.h
///\brief	Definition of types used in MPC
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author      Keith François
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////


#include <Eigen/Dense>

#include <iostream>

#include <mpc-walkgen/common/sharedpgtypes.h>

namespace MPCWalkgen{

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

}


#endif // MPC_WALKGEN_COMMON_TYPE_H
