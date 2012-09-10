#include <mpc-walkgen/rigid-body-system.h>
#include <mpc-walkgen/com-body.h>//TODO: These two to .h?
#include <mpc-walkgen/foot-body.h>

using namespace MPCWalkgen;
using namespace Eigen;

RigidBodySystem::RigidBodySystem() { }

RigidBodySystem::~RigidBodySystem() {
  if (com_ != 0x0)
    delete com_;

  if (foot_left_ != 0x0)
    delete foot_left_;

  if (foot_right_ != 0x0)
    delete foot_right_;
}

void RigidBodySystem::Init(const MPCData * mpc_parameters_p) {
  mpc_parameters_p_ = mpc_parameters_p;
}

void RigidBodySystem::Init(const RobotData &data_robot, const Interpolation *interpolation) {//TODO: Remove object data_robot
  data_robot_ = data_robot;
  

  com_ = new CoMBody(mpc_parameters_p_, &data_robot_);
  foot_left_ = new FootBody(mpc_parameters_p_, &data_robot_,  LEFT);
  foot_right_ = new FootBody(mpc_parameters_p_, &data_robot_, RIGHT);

  currentSupport_.phase = DS;
  currentSupport_.foot = LEFT;
  currentSupport_.time_limit = 1e9;
  currentSupport_.nbStepsLeft = 1;
  currentSupport_.state_changed = true;
  currentSupport_.x = data_robot.leftFootPos(0);
  currentSupport_.y = data_robot.leftFootPos(1);
  currentSupport_.yaw = 0.0;
  currentSupport_.yawTrunk = 0.0;
  currentSupport_.start_time = 0.0;

  ComputeDynamics();
}

void RigidBodySystem::ComputeDynamics() {
  com_->ComputeDynamics();
  foot_left_->ComputeDynamics();
  foot_right_->ComputeDynamics();
}

void RigidBodySystem::interpolateBodies(MPCSolution &solution, double currentTime, const Reference &velRef){
  com_->Interpolate(solution, currentTime, velRef);
  foot_left_->Interpolate(solution, currentTime, velRef);
  foot_right_->Interpolate(solution, currentTime, velRef);
}

void RigidBodySystem::UpdateState(const MPCSolution &solution) {
  BodyState foot_left, foot_right, com;
  // TODO: State updates can/should be done locally in RigidBody
  int next_sample = mpc_parameters_p_->num_samples_act() - 1;
  for (int i = 0; i < 3; ++i){
    const MPCSolution::State &currentState = solution.state_vec[i];
    foot_left.x(i) = currentState.leftFootTrajX_(next_sample);
    foot_left.y(i) = currentState.leftFootTrajY_(next_sample);
    foot_left.z(i) = currentState.leftFootTrajZ_(next_sample);
    foot_left.yaw(i) = currentState.leftFootTrajYaw_(next_sample);

    foot_right.x(i) = currentState.rightFootTrajX_(next_sample);
    foot_right.y(i) = currentState.rightFootTrajY_(next_sample);
    foot_right.z(i) = currentState.rightFootTrajZ_(next_sample);
    foot_right.yaw(i) = currentState.rightFootTrajYaw_(next_sample);
  }
  // TODO: Not necessary if feedback == true
  com.x(0) = solution.com_act.pos.x_vec(next_sample);
  com.y(0) = solution.com_act.pos.y_vec(next_sample);
  com.x(1) = solution.com_act.vel.x_vec(next_sample);
  com.y(1) = solution.com_act.vel.y_vec(next_sample);
  com.x(2) = solution.com_act.acc.x_vec(next_sample);
  com.y(2) = solution.com_act.acc.y_vec(next_sample);

  // TODO: Temporary solutions
  com.yaw(0) = solution.state_vec[0].trunkYaw_(next_sample);
  com.yaw(1) = solution.state_vec[1].trunkYaw_(next_sample);
  com.yaw(2) = solution.state_vec[2].trunkYaw_(next_sample);
  com.z(0) = data_robot_.com(2);
  com.z(1) = 0.0;
  com.z(2) = 0.0;

  com_->state(com);
  foot_left_->state(foot_left);
  foot_right_->state(foot_right);

}

void RigidBodySystem::setSelectionNumber(double firstSamplingPeriod){
  com_->setSelectionNumber(firstSamplingPeriod);
  foot_left_->setSelectionNumber(firstSamplingPeriod);
  foot_right_->setSelectionNumber(firstSamplingPeriod);
}

RigidBody *RigidBodySystem::body(BodyType type){
  switch(type){
    case COM:
      return com_;
    case LEFT_FOOT:
      return foot_left_;
    default:
      return foot_right_;
  }
}

const RigidBody *RigidBodySystem::body(BodyType type) const{
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
        hull = data_robot_.leftFootHull;
      }else{
        hull = data_robot_.rightFootHull;
      }
      break;
    case CoPHull:
      if (prwSupport.foot == LEFT){
        if (prwSupport.phase == SS){
          hull = data_robot_.CoPLeftSSHull;
        }else{
          hull = data_robot_.CoPLeftDSHull;
        }
      }else{
        if (prwSupport.phase==SS){
          hull = data_robot_.CoPRightSSHull;
        }else{
          hull =  data_robot_.CoPRightDSHull;
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
