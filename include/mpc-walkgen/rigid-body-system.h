#pragma once
#ifndef MPC_WALKGEN_RIGID_BODY_SYSTEM_H
#define MPC_WALKGEN_RIGID_BODY_SYSTEM_H

#include <mpc-walkgen/types.h>
#include <mpc-walkgen/rigid-body.h>

#include <Eigen/Dense>
#include <vector>

namespace MPCWalkgen{
class RigidBodySystem{

	//
	// Public methods:
	//
public:
	RigidBodySystem();
	~RigidBodySystem();

	void Init(const MPCParameters *mpc_parameters);

	void Init(const RobotData &robot_data);

	void Interpolate(MPCSolution &solution, double current_time, const Reference &ref);

	void UpdateState(const MPCSolution &solution);

	void ComputeDynamics();

	// Returns
	void GetState(CommonVectorType &state_x, CommonVectorType &state_y);

	void GetConvexHull(ConvexHull &hull, HullType type, const SupportState &prwSupport) const;

	//
	// Private data members:
	//
private:
	const MPCParameters *mpc_parameters_;
	RobotData robot_data_;

	DynamicsBuilder dynamics_builder_;

	RigidBody *com_;
	RigidBody *left_foot_;
	RigidBody *right_foot_;

	SupportState current_support_;

#include <mpc-walkgen/rigid-body-system-inl.h>
};
}
#endif // MPC_WALKGEN_RIGID_BODY_SYSTEM_H
