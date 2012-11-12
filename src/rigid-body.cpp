#include <mpc-walkgen/rigid-body.h>
#include <mpc-walkgen/debug.h>

#include <iostream>
#include <stdio.h>

using namespace MPCWalkgen;


RigidBody::RigidBody():mpc_parameters_(NULL)
,robot_data_p_(NULL)
,dyn_build_p_(NULL)
{}

RigidBody::~RigidBody() {}

void RigidBody::Init(const MPCParameters *mpc_parameters_p) {
	mpc_parameters_ = mpc_parameters_p;

	int nbdynamics = mpc_parameters_->GetNumRecomputations();
	dynamics_qp_vec_.resize(nbdynamics);

	int num_samples = mpc_parameters_->num_samples_act();
	int num_unst_modes = 1;//TODO: unstable modes
	motion_act_.SetZero(num_samples, num_unst_modes);
}

void RigidBody::Init(const RobotData *robot_data_p) {
	robot_data_p_ = robot_data_p;
}

void RigidBody::Init(DynamicsBuilder *dyn_build_p) {
	dyn_build_p_ = dyn_build_p;
}

void RigidBody::ComputeDynamics(SystemOrder dynamics_order) {
	assert(state_.z(0) > kEps);

	int num_samples = mpc_parameters_->num_samples_horizon;
	double sp_first = mpc_parameters_->period_mpcsample;
	double sp_rest = mpc_parameters_->period_qpsample;
	int num_dynamics = mpc_parameters_->GetNumRecomputations();
	double com_height = state_.z(0);

	std::vector<LinearDynamics>::iterator dyn_it = dynamics_qp_vec_.begin();
	for (int k = 0; k < num_dynamics; ++k) {
		sp_first = mpc_parameters_->period_mpcsample * (k+1);
		dyn_build_p_->Build(dynamics_order, *dyn_it, com_height, sp_first, sp_rest, num_samples);
		++dyn_it;
	}

	num_samples = mpc_parameters_->num_samples_act();
	sp_first = mpc_parameters_->period_actsample;
	sp_rest = mpc_parameters_->period_actsample;
	dyn_build_p_->Build(dynamics_order, dynamics_act_, com_height, sp_first, sp_rest, num_samples);
}
