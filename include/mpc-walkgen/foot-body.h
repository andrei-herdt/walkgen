#pragma once
#ifndef MPC_WALKGEN_FOOT_BODY_H
#define MPC_WALKGEN_FOOT_BODY_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	com-body.h
///\brief	A class to store CoM rigid body
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author      Keith François
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/types.h>
#include <mpc-walkgen/rigid-body.h>

#include <Eigen/Dense>

namespace MPCWalkgen{
	class FootBody: public RigidBody{

		//
		// Public methods:
		//
	public:
		FootBody(Foot type);
		virtual ~FootBody();

		virtual void Interpolate(MPCSolution &solution, double current_time, const Reference &ref);

		//
		// Private methods:
		//
	private:
		void InterpolatePolynomial(
			MPCSolution &solution, 
			CommonVectorType &pos_vec,
			CommonVectorType &vel_vec,
			CommonVectorType &acc_vec,
			int num_samples, 
			double T,
			const Eigen::Vector3d &state_vec,
			const Eigen::Vector3d &nextSupportFootState);

		//
		// Private data members:
		//
	private:
		Foot which_;
	};
}

#endif // MPC_WALKGEN_FOOT_BODY_H
