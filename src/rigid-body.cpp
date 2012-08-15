#include <mpc-walkgen/rigid-body.h>

using namespace MPCWalkgen;


RigidBody::RigidBody(const MPCData * data_mpc,
                     const RobotData * data_robot)
                     :data_mpc_(data_mpc)
                     ,robotData_(data_robot)
{}

RigidBody::~RigidBody()
{}

void RigidBody::ComputeDynamics() {

  int nbsamples = data_mpc_->nbsamples_qp;
  double sp_first = data_mpc_->period_mpcsample;
  double sp_rest = data_mpc_->period_qpsample;
  int nbdynamics = data_mpc_->nbFeedbackSamplesStandard();
  dynamics_qp_vec_.resize(nbdynamics);
  std::vector<LinearDynamics>::iterator dyn_it = dynamics_qp_vec_.begin();
  for (int k = 0; k < nbdynamics; ++k) {
    sp_first = data_mpc_->period_mpcsample * (k+1);
    ComputeDynamicsMatrices(dyn_it->pos, sp_first, sp_rest, data_mpc_->nbsamples_qp, POSITION);
    ComputeDynamicsMatrices(dyn_it->vel, sp_first, sp_rest, data_mpc_->nbsamples_qp, VELOCITY);
    ComputeDynamicsMatrices(dyn_it->acc, sp_first, sp_rest, data_mpc_->nbsamples_qp, ACCELERATION);
    ComputeDynamicsMatrices(dyn_it->jerk, sp_first, sp_rest, data_mpc_->nbsamples_qp, JERK);
    ComputeDynamicsMatrices(dyn_it->cop, sp_first, sp_rest, data_mpc_->nbsamples_qp, COP);
    ++dyn_it;
  }

  nbsamples = data_mpc_->num_samples_act();
  sp_first = data_mpc_->period_actsample;
  sp_rest = data_mpc_->period_actsample;
  ComputeDynamicsMatrices(dynamics_act_.pos, sp_first, sp_rest, nbsamples, POSITION);
  ComputeDynamicsMatrices(dynamics_act_.vel, sp_first, sp_rest, nbsamples, VELOCITY);
  ComputeDynamicsMatrices(dynamics_act_.acc, sp_first, sp_rest, nbsamples, ACCELERATION);
  ComputeDynamicsMatrices(dynamics_act_.cop, sp_first, sp_rest, nbsamples, COP);
}

void RigidBody::setSelectionNumber(double firstSamplingPeriod){
  dynamics_iter_ = (int)round(firstSamplingPeriod / data_mpc_->period_mpcsample) - 1;
}
