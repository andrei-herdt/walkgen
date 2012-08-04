#include "rigid-body-system.h"
#include "rigid-bodies/com-body.h"
#include "rigid-bodies/foot-body.h"

using namespace MPCWalkgen;

using namespace Eigen;

RigidBodySystem::RigidBodySystem(const MPCData *generalData)
:generalData_(generalData)
,robot_data_() {

}

RigidBodySystem::~RigidBodySystem() {
  if (com_ != 0x0)
    delete com_;

  if (foot_left_ != 0x0)
    delete foot_left_;
  
  if (foot_right_ != 0x0)
    delete foot_right_;
}

void RigidBodySystem::Init(const RobotData &robot_data, const Interpolation *interpolation) {//TODO: Remove object robot_data
  robot_data_ = robot_data;

  com_ = new CoMBody(generalData_, &robot_data_, interpolation);
  foot_left_ = new FootBody(generalData_, &robot_data_, interpolation, LEFT);
  foot_right_ = new FootBody(generalData_, &robot_data_, interpolation, RIGHT);

  currentSupport_.phase = DS;
  currentSupport_.foot = LEFT;
  currentSupport_.timeLimit = 1e9;
  currentSupport_.nbStepsLeft = 1;
  currentSupport_.stateChanged = true;
  currentSupport_.x = robot_data.leftFootPos(0);
  currentSupport_.y = robot_data.leftFootPos(1);
  currentSupport_.yaw = 0.0;
  currentSupport_.yawTrunk = 0.0;
  currentSupport_.startTime = 0.0;
}

void RigidBodySystem::computeDynamics() {
  com_->computeDynamics();
  foot_left_->computeDynamics();
  foot_right_->computeDynamics();
}

void RigidBodySystem::interpolateBodies(MPCSolution &solution, double currentTime, const Reference &velRef){
  com_->interpolate(solution, currentTime, velRef);
  foot_left_->interpolate(solution, currentTime, velRef);
  foot_right_->interpolate(solution, currentTime, velRef);
}

void RigidBodySystem::updateBodyState(const MPCSolution &solution){
  int nextCurrentState = (int)round(generalData_->period_mpcsample / generalData_->period_actsample)-1;

  BodyState leftFoot, rightFoot, CoM;

  for (int i = 0; i < 3; ++i){
    const MPCSolution::State &currentState = solution.state_vec[i];
    leftFoot.x(i) = currentState.leftFootTrajX_(nextCurrentState);
    leftFoot.y(i) = currentState.leftFootTrajY_(nextCurrentState);
    leftFoot.z(i) = currentState.leftFootTrajZ_(nextCurrentState);
    leftFoot.yaw(i) = currentState.leftFootTrajYaw_(nextCurrentState);

    rightFoot.x(i) = currentState.rightFootTrajX_(nextCurrentState);
    rightFoot.y(i) = currentState.rightFootTrajY_(nextCurrentState);
    rightFoot.z(i) = currentState.rightFootTrajZ_(nextCurrentState);
    rightFoot.yaw(i) = currentState.rightFootTrajYaw_(nextCurrentState);

    CoM.x(i) = currentState.CoMTrajX_(nextCurrentState);
    CoM.y(i) = currentState.CoMTrajY_(nextCurrentState);
    CoM.yaw(i) = currentState.trunkYaw_(nextCurrentState);
  }
  CoM.z(0) = robot_data_.com(2);
  CoM.z(1) = 0.0;
  CoM.z(2) = 0.0;

  com_->state(CoM);
  foot_left_->state(leftFoot);
  foot_right_->state(rightFoot);

}

void RigidBodySystem::setSelectionNumber(double firstSamplingPeriod){
  com_->setSelectionNumber(firstSamplingPeriod);
  foot_left_->setSelectionNumber(firstSamplingPeriod);
  foot_right_->setSelectionNumber(firstSamplingPeriod);
}

RigidBody * RigidBodySystem::body(BodyType type){
  switch(type){
    case COM:
      return com_;
    case LEFT_FOOT:
      return foot_left_;
    default:
      return foot_right_;
  }
}

const RigidBody * RigidBodySystem::body(BodyType type) const{
  switch(type){
    case COM:
      return com_;
    case LEFT_FOOT:
      return foot_left_;
    default:
      return foot_right_;
  }
}

void RigidBodySystem::convexHull(ConvexHull &hull, HullType type, const SupportState &prwSupport, bool computeLinearSystem, bool rotateHull) const {
  switch (type){
    case FootHull:
      if (prwSupport.foot == LEFT){
        hull = robot_data_.leftFootHull;
      }else{
        hull = robot_data_.rightFootHull;
      }
      break;
    case CoPHull:
      if (prwSupport.foot == LEFT){
        if (prwSupport.phase == SS){
          hull = robot_data_.CoPLeftSSHull;
        }else{
          hull = robot_data_.CoPLeftDSHull;
        }
      }else{
        if (prwSupport.phase==SS){
          hull = robot_data_.CoPRightSSHull;
        }else{
          hull =  robot_data_.CoPRightDSHull;
        }
      }
      break;
  }

  if (rotateHull){
    hull.rotate(prwSupport.yaw);
  }

  if (computeLinearSystem){
    hull.computeLinearSystem(prwSupport.foot);
  }
}
