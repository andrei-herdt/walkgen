#pragma once
#ifndef MPC_WALKGEN_FOOT_BODY_H
#define MPC_WALKGEN_FOOT_BODY_H

#include <mpc-walkgen/types.h>
#include <mpc-walkgen/rigid-body.h>

namespace MPCWalkgen{
	class FootBody: public RigidBody{

		//
		// Public methods:
		//
	public:
		FootBody(Foot type);
		virtual ~FootBody();

		virtual void Interpolate(MPCSolution &solution, double current_time, const Reference &ref);

		inline const Vector3D &local_ankle_pos() {
			return robot_data_->left_foot.ankle_pos_local;
		}

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
