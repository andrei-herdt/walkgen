#include <mpc-walkgen/rigid-body.h>

#include <iostream>
#include <stdio.h>

using namespace MPCWalkgen;


RigidBody::RigidBody():mpc_parameters_p_(NULL)
,robot_data_p_(NULL)
,dyn_build_p_(NULL)
,dynamics_index_(0)
{}

RigidBody::~RigidBody() {}

void RigidBody::Init(const MPCParameters *mpc_parameters_p) {
	mpc_parameters_p_ = mpc_parameters_p;

	int nbdynamics = mpc_parameters_p_->GetNumRecomputations();
	dynamics_qp_vec_.resize(nbdynamics);

	int num_samples = mpc_parameters_p_->num_samples_act();
	motion_act_.SetZero(num_samples);
}

void RigidBody::Init(const RobotData *robot_data_p) {
	robot_data_p_ = robot_data_p;
}

void RigidBody::Init(DynamicsBuilder *dyn_build_p) {
	dyn_build_p_ = dyn_build_p;
}

void RigidBody::ComputeDynamics(SystemOrder dynamics_order) {
	assert(state_.z(0) > kEps);

	int num_samples = mpc_parameters_p_->num_samples_horizon;
	double sp_first = mpc_parameters_p_->period_mpcsample;
	double sp_rest = mpc_parameters_p_->period_qpsample;
	int num_dynamics = mpc_parameters_p_->GetNumRecomputations();
	double height = state_.z(0);

	std::vector<LinearDynamics>::iterator dyn_it = dynamics_qp_vec_.begin();
	for (int k = 0; k < num_dynamics; ++k) {
		sp_first = mpc_parameters_p_->period_mpcsample * (k+1);
		std::cout << "sp_first: " << sp_first << std::endl;
		dyn_build_p_->Build(dynamics_order, *dyn_it, robot_data_p_->com(2), sp_first, sp_rest, num_samples);
		++dyn_it;
	}

	num_samples = mpc_parameters_p_->num_samples_act();
	sp_first = mpc_parameters_p_->period_actsample;
	sp_rest = mpc_parameters_p_->period_actsample;
	dyn_build_p_->Build(dynamics_order, dynamics_act_, robot_data_p_->com(2), sp_first, sp_rest, num_samples);
}

void RigidBody::ComputeDynamicsIndex(double first_sampling_period){
	dynamics_index_ = mpc_parameters_p_->GetMPCSamplesLeft(first_sampling_period);
}
