#include <mpc-walkgen/rigid-body.h>

#include <iostream>
#include <stdio.h>

using namespace MPCWalkgen;


RigidBody::RigidBody():mpc_parameters_(NULL)
,robot_data_p_(NULL)
,dyn_build_p_(NULL)
,dynamics_index_(0)
{}

RigidBody::~RigidBody() {}

void RigidBody::Init(const MPCParameters *mpc_parameters_p) {
	mpc_parameters_ = mpc_parameters_p;

	int nbdynamics = mpc_parameters_->nbFeedbackSamplesStandard();
	dynamics_qp_vec_.resize(nbdynamics);

	int num_samples = mpc_parameters_->num_samples_act();
	motion_act_.Resize(num_samples);
}

void RigidBody::Init(const RobotData *robot_data_p) {
	robot_data_p_ = robot_data_p;
}

void RigidBody::Init(DynamicsBuilder *dyn_build_p) {
	dyn_build_p_ = dyn_build_p;
}

void RigidBody::ComputeDynamics(DynamicsOrder dynamics_order) {
	assert(state_.z(0) > kEps);

	int num_samples = mpc_parameters_->num_samples_horizon;
	double sp_first = mpc_parameters_->period_mpcsample;
	double sp_rest = mpc_parameters_->period_qpsample;
	int nbdynamics = mpc_parameters_->nbFeedbackSamplesStandard();
	double height = state_.z(0);

	std::vector<LinearDynamics>::iterator dyn_it = dynamics_qp_vec_.begin();
	for (int k = 0; k < nbdynamics; ++k) {
		sp_first = mpc_parameters_->period_mpcsample * (k+1);
		dyn_build_p_->Build(dynamics_order, *dyn_it, robot_data_p_->com(2), sp_first, sp_rest, num_samples);
		++dyn_it;
	}

	num_samples = mpc_parameters_->num_samples_act();
	sp_first = mpc_parameters_->period_actsample;
	sp_rest = mpc_parameters_->period_actsample;
	dyn_build_p_->Build(dynamics_order, dynamics_act_, robot_data_p_->com(2), sp_first, sp_rest, num_samples);
}

void RigidBody::SetSelectionNumber(double firstSamplingPeriod){
	dynamics_index_ = (int)round(firstSamplingPeriod / mpc_parameters_->period_mpcsample) - 1;
}
