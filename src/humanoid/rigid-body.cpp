#include "rigid-body.h"

using namespace MPCWalkgen;


RigidBody::RigidBody(const MPCData * generalData,
                     const RobotData * robotData, const Interpolation * interpolation)
  :generalData_(generalData)
  ,robotData_(robotData)
  ,interpolation_(interpolation)
{}

RigidBody::~RigidBody()
{}

const LinearDynamics & RigidBody::dynamics(DynamicMatrixType type) const{
  switch (type){
    case posDynamic:
      return pos_dynamics_vec_[matrixNumber_];
    case velDynamic:
      return vel_dynamics_vec_[matrixNumber_];
    case accDynamic:
      return acc_dynamics_vec_[matrixNumber_];
    case jerkDynamic:
      return jerk_dynamics_vec_[matrixNumber_];
    case copDynamic:
      return cop_dynamics_vec_[matrixNumber_];
    case interpolationPos:
      return posInterpol_;
    case interpolationVel:
      return velInterpol_;
    case interpolationAcc:
      return accInterpol_;
    default:
      return copInterpol_;
    }
}

void RigidBody::setSelectionNumber(double firstSamplingPeriod){
  matrixNumber_ = (int)round(firstSamplingPeriod / generalData_->period_mpcsample) - 1;
}

void RigidBody::computeDynamics(){

  int vecSize = generalData_->nbFeedbackSamplesStandard();

  pos_dynamics_vec_.resize(vecSize);
  vel_dynamics_vec_.resize(vecSize);
  acc_dynamics_vec_.resize(vecSize);
  jerk_dynamics_vec_.resize(vecSize);
  cop_dynamics_vec_.resize(vecSize);

  for (int k = 0; k < vecSize; ++k) {
      double S = generalData_->period_mpcsample * (k+1);
      computeDynamicsMatrices(pos_dynamics_vec_[k], S,
                              generalData_->period_qpsample, generalData_->nbsamples_qp, posDynamic);
      computeDynamicsMatrices(vel_dynamics_vec_[k], S,
                              generalData_->period_qpsample, generalData_->nbsamples_qp, velDynamic);
      computeDynamicsMatrices(acc_dynamics_vec_[k], S,
                              generalData_->period_qpsample, generalData_->nbsamples_qp, accDynamic);
      computeDynamicsMatrices(jerk_dynamics_vec_[k], S,
                              generalData_->period_qpsample, generalData_->nbsamples_qp, jerkDynamic);
      computeDynamicsMatrices(cop_dynamics_vec_[k], S,
                              generalData_->period_qpsample, generalData_->nbsamples_qp, copDynamic);

    }

  int nbSamplingSim = generalData_->nbSamplesControl();
  computeDynamicsMatrices(posInterpol_, generalData_->period_actsample,
                          generalData_->period_actsample, nbSamplingSim, posDynamic);

  computeDynamicsMatrices(velInterpol_, generalData_->period_actsample,
                          generalData_->period_actsample, nbSamplingSim, velDynamic);

  computeDynamicsMatrices(accInterpol_, generalData_->period_actsample,
                          generalData_->period_actsample, nbSamplingSim, accDynamic);

  computeDynamicsMatrices(copInterpol_, generalData_->period_actsample,
                          generalData_->period_actsample, nbSamplingSim, copDynamic);
}

