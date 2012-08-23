#include <mpc-walkgen/rigid-body.h>

using namespace MPCWalkgen;


RigidBody::RigidBody(const MPCData * mpc_parameters, const RobotData * data_robot)
                     :mpc_parameters_(mpc_parameters)
                     ,robotData_(data_robot)
{}

RigidBody::~RigidBody()
{}

void RigidBody::ComputeDynamics() {

  int nbsamples = mpc_parameters_->nbsamples_qp;
  double sp_first = mpc_parameters_->period_mpcsample;
  double sp_rest = mpc_parameters_->period_qpsample;
  int nbdynamics = mpc_parameters_->nbFeedbackSamplesStandard();
  dynamics_qp_vec_.resize(nbdynamics);
  std::vector<LinearDynamics>::iterator dyn_it = dynamics_qp_vec_.begin();
  for (int k = 0; k < nbdynamics; ++k) {
    sp_first = mpc_parameters_->period_mpcsample * (k+1);
    ComputeDynamicsMatrices(dyn_it->pos, sp_first, sp_rest, mpc_parameters_->nbsamples_qp, POSITION);
    ComputeDynamicsMatrices(dyn_it->vel, sp_first, sp_rest, mpc_parameters_->nbsamples_qp, VELOCITY);
    ComputeDynamicsMatrices(dyn_it->acc, sp_first, sp_rest, mpc_parameters_->nbsamples_qp, ACCELERATION);
    ComputeDynamicsMatrices(dyn_it->jerk, sp_first, sp_rest, mpc_parameters_->nbsamples_qp, JERK);
    ComputeDynamicsMatrices(dyn_it->cop, sp_first, sp_rest, mpc_parameters_->nbsamples_qp, COP);
    ++dyn_it;
  }

  nbsamples = mpc_parameters_->num_samples_act();
  sp_first = mpc_parameters_->period_actsample;
  sp_rest = mpc_parameters_->period_actsample;
  ComputeDynamicsMatrices(dynamics_act_.pos, sp_first, sp_rest, nbsamples, POSITION);
  ComputeDynamicsMatrices(dynamics_act_.vel, sp_first, sp_rest, nbsamples, VELOCITY);
  ComputeDynamicsMatrices(dynamics_act_.acc, sp_first, sp_rest, nbsamples, ACCELERATION);
  ComputeDynamicsMatrices(dynamics_act_.cop, sp_first, sp_rest, nbsamples, COP);
}

void RigidBody::setSelectionNumber(double firstSamplingPeriod){
  dynamics_iter_ = (int)round(firstSamplingPeriod / mpc_parameters_->period_mpcsample) - 1;
}
