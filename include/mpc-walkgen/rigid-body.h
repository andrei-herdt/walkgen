#pragma once
#ifndef MPC_WALKGEN_RIGID_BODY_H
#define MPC_WALKGEN_RIGID_BODY_H

#include <mpc-walkgen/types.h>
#include <mpc-walkgen/interpolation.h>
#include <mpc-walkgen/dynamics-builder.h>
#include <vector>

namespace MPCWalkgen{
class RigidBody{

	//
	// Public methods:
	//
public:
	RigidBody();
	virtual ~RigidBody();

	void Init(const MPCParameters *mpc_parameters_p);
	void Init(const RobotData *data_robot_p);
	void Init(DynamicsBuilder *dyn_build_p);

	void ComputeDynamics(SystemOrder system_order);

	virtual void Interpolate(MPCSolution &solution, double current_time, const Reference &ref)=0;

	/// \name Accessors and mutators
	/// \{
	inline const BodyState &state() const {return state_;}
	inline BodyState &state(){return state_;}

	inline const Wrench &force_sensor() const {return wrench_;};
	inline Wrench &force_sensor() {return wrench_;};

	inline void state(const BodyState &s){state_ = s;}

	inline const std::vector<LinearDynamics> &dynamics_qp() const { return dynamics_qp_vec_;};
	inline const LinearDynamics &dynamics_act() const {return dynamics_act_;};

	inline Motion &motion_act() {return motion_act_;};
	inline Motion &motion_prw() {return motion_prw_;};

	/// \}

protected:
	const MPCParameters *mpc_parameters_;
	const RobotData *robot_data_; //TODO: Maybe should not be here
	DynamicsBuilder *dyn_build_p_;

	Interpolation interpolation_;

	BodyState state_;

	Wrench wrench_;

	std::vector<LinearDynamics> dynamics_qp_vec_; // \brief Dynamics sampled with the QP sampling rate

	LinearDynamics dynamics_act_;    // \brief Dynamics sampled with the actuator sampling rate


	Motion motion_act_, motion_prw_;

};
}

#endif // MPC_WALKGEN_RIGID_BODY_H
