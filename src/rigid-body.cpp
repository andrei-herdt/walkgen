#include <mpc-walkgen/rigid-body.h>

#include <iostream>
#include <stdio.h>

using namespace MPCWalkgen;


RigidBody::RigidBody() {}

RigidBody::~RigidBody() {}

void RigidBody::Init(const MPCData *mpc_parameters_p) {
  mpc_parameters_ = mpc_parameters_p;
  
  int nbdynamics = mpc_parameters_->nbFeedbackSamplesStandard();
  dynamics_qp_vec_.resize(nbdynamics);

  int num_samples = mpc_parameters_->num_samples_act();
  motion_act_.resize(num_samples);
}

void RigidBody::Init(const RobotData *robot_data_p) {
  robot_data_p_ = robot_data_p;
}

void RigidBody::Init(DynamicsBuilder *dyn_build_p) {
  dyn_build_p_ = dyn_build_p;
}

void RigidBody::ComputeDynamics() {
  int num_samples = mpc_parameters_->nbsamples_qp;
  double sp_first = mpc_parameters_->period_mpcsample;
  double sp_rest = mpc_parameters_->period_qpsample;
  int nbdynamics = mpc_parameters_->nbFeedbackSamplesStandard();

  std::vector<LinearDynamics>::iterator dyn_it = dynamics_qp_vec_.begin();
  for (int k = 0; k < nbdynamics; ++k) {
    sp_first = mpc_parameters_->period_mpcsample * (k+1);
    ComputeDynamicsMatrices(dyn_it->pos, sp_first, sp_rest, num_samples, POSITION);
    ComputeDynamicsMatrices(dyn_it->vel, sp_first, sp_rest, num_samples, VELOCITY);
    ComputeDynamicsMatrices(dyn_it->acc, sp_first, sp_rest, num_samples, ACCELERATION);
    ComputeDynamicsMatrices(dyn_it->jerk, sp_first, sp_rest, num_samples, JERK);
    ComputeDynamicsMatrices(dyn_it->cop, sp_first, sp_rest, num_samples, COP);
    ++dyn_it;
  }

  num_samples = mpc_parameters_->num_samples_act();
  sp_first = mpc_parameters_->period_actsample;
  sp_rest = mpc_parameters_->period_actsample;
  ComputeDynamicsMatrices(dynamics_act_.pos, sp_first, sp_rest, num_samples, POSITION);
  ComputeDynamicsMatrices(dynamics_act_.vel, sp_first, sp_rest, num_samples, VELOCITY);
  ComputeDynamicsMatrices(dynamics_act_.acc, sp_first, sp_rest, num_samples, ACCELERATION);
  ComputeDynamicsMatrices(dynamics_act_.cop, sp_first, sp_rest, num_samples, COP);
}

void RigidBody::ComputeDynamics(DynamicsType dynamics_type) {
  assert(state_.z(0) > kEps);

  int num_samples = mpc_parameters_->nbsamples_qp;
  double sp_first = mpc_parameters_->period_mpcsample;
  double sp_rest = mpc_parameters_->period_qpsample;
  int nbdynamics = mpc_parameters_->nbFeedbackSamplesStandard();
  double height = state_.z(0);

  std::vector<LinearDynamics>::iterator dyn_it = dynamics_qp_vec_.begin();
  for (int k = 0; k < nbdynamics; ++k) {
    sp_first = mpc_parameters_->period_mpcsample * (k+1);
    dyn_build_p_->Build(dynamics_type, *dyn_it, robot_data_p_->com(2), sp_first, sp_rest, num_samples);
    ++dyn_it;
  }

  num_samples = mpc_parameters_->num_samples_act();
  sp_first = mpc_parameters_->period_actsample;
  sp_rest = mpc_parameters_->period_actsample;
  dyn_build_p_->Build(dynamics_type, dynamics_act_, robot_data_p_->com(2), sp_first, sp_rest, num_samples);
}

void RigidBody::setSelectionNumber(double firstSamplingPeriod){
  dynamics_iter_ = (int)round(firstSamplingPeriod / mpc_parameters_->period_mpcsample) - 1;
}
