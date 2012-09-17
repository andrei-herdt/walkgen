#include <mpc-walkgen/rigid-body-system.h>
#include <mpc-walkgen/com-body.h>//TODO: These two to .h?
#include <mpc-walkgen/foot-body.h>
#include <limits>
#include <iostream>

using namespace MPCWalkgen;
using namespace Eigen;

RigidBodySystem::RigidBodySystem() { 
	com_ = new CoMBody();
	left_foot_ = new FootBody(LEFT);
	right_foot_ = new FootBody(RIGHT);
}

RigidBodySystem::~RigidBodySystem() {
	if (com_ != 0x0)
		delete com_;

	if (left_foot_ != 0x0)
		delete left_foot_;

	if (right_foot_ != 0x0)
		delete right_foot_;
}

void RigidBodySystem::Init(const MPCData *mpc_parameters_p) {
	mpc_parameters_p_ = mpc_parameters_p;

	com_->Init(mpc_parameters_p_);
	left_foot_->Init(mpc_parameters_p_);
	right_foot_->Init(mpc_parameters_p_);
}

void RigidBodySystem::Init(const RobotData &data_robot) {//TODO: Remove object data_robot
	data_robot_ = data_robot;

	com_->Init(&data_robot);
	left_foot_->Init(&data_robot);
	right_foot_->Init(&data_robot);

	currentSupport_.phase = DS;
	currentSupport_.foot = LEFT;
	currentSupport_.time_limit = 1e9;
	currentSupport_.nbStepsLeft = 1;
	currentSupport_.state_changed = true;
	currentSupport_.x = data_robot.leftFoot.position[0];
	currentSupport_.y = data_robot.leftFoot.position[1];
	currentSupport_.yaw = 0.0;
	currentSupport_.yawTrunk = 0.0;
	currentSupport_.start_time = 0.0;
}

void RigidBodySystem::ComputeDynamics() {
	com_->ComputeDynamics();
	left_foot_->ComputeDynamics();
	right_foot_->ComputeDynamics();
}

void RigidBodySystem::Interpolate(MPCSolution &solution, double currentTime, const Reference &velRef){
	com_->Interpolate(solution, currentTime, velRef);
	left_foot_->Interpolate(solution, currentTime, velRef);
	right_foot_->Interpolate(solution, currentTime, velRef);
}

void RigidBodySystem::UpdateState(const MPCSolution &solution) {


  int next_sample = mpc_parameters_p_->num_samples_act() - 1;
	left_foot_->state().x(POSITION) = left_foot_->motion_act().pos.x_vec[next_sample];
	left_foot_->state().y(POSITION) = left_foot_->motion_act().pos.y_vec[next_sample];
	left_foot_->state().z(POSITION) = left_foot_->motion_act().pos.z_vec[next_sample];
	left_foot_->state().yaw(POSITION) = left_foot_->motion_act().pos.yaw_vec[next_sample];
	left_foot_->state().x(VELOCITY) = left_foot_->motion_act().vel.x_vec[next_sample];
	left_foot_->state().y(VELOCITY) = left_foot_->motion_act().vel.y_vec[next_sample];
	left_foot_->state().z(VELOCITY) = left_foot_->motion_act().vel.z_vec[next_sample];
	left_foot_->state().yaw(VELOCITY) = left_foot_->motion_act().vel.yaw_vec[next_sample];
	left_foot_->state().x(ACCELERATION) = left_foot_->motion_act().acc.x_vec[next_sample];
	left_foot_->state().y(ACCELERATION) = left_foot_->motion_act().acc.y_vec[next_sample];
	left_foot_->state().z(ACCELERATION) = left_foot_->motion_act().acc.z_vec[next_sample];
	left_foot_->state().yaw(ACCELERATION) = left_foot_->motion_act().acc.yaw_vec[next_sample];

	right_foot_->state().x(POSITION) = right_foot_->motion_act().pos.x_vec[next_sample];
	right_foot_->state().y(POSITION) = right_foot_->motion_act().pos.y_vec[next_sample];
	right_foot_->state().z(POSITION) = right_foot_->motion_act().pos.z_vec[next_sample];
	right_foot_->state().yaw(POSITION) = right_foot_->motion_act().pos.yaw_vec[next_sample];
	right_foot_->state().x(VELOCITY) = right_foot_->motion_act().vel.x_vec[next_sample];
	right_foot_->state().y(VELOCITY) = right_foot_->motion_act().vel.y_vec[next_sample];
	right_foot_->state().z(VELOCITY) = right_foot_->motion_act().vel.z_vec[next_sample];
	right_foot_->state().yaw(VELOCITY) = right_foot_->motion_act().vel.yaw_vec[next_sample];
	right_foot_->state().x(ACCELERATION) = right_foot_->motion_act().acc.x_vec[next_sample];
	right_foot_->state().y(ACCELERATION) = right_foot_->motion_act().acc.y_vec[next_sample];
	right_foot_->state().z(ACCELERATION) = right_foot_->motion_act().acc.z_vec[next_sample];
	right_foot_->state().yaw(ACCELERATION) = right_foot_->motion_act().acc.yaw_vec[next_sample];


	BodyState com;
	// TODO: Not necessary if feedback == true
	com.x(0) = solution.com_act.pos.x_vec[next_sample];
	com.y(0) = solution.com_act.pos.y_vec[next_sample];
	com.x(1) = solution.com_act.vel.x_vec[next_sample];
	com.y(1) = solution.com_act.vel.y_vec[next_sample];
	com.x(2) = solution.com_act.acc.x_vec[next_sample];
	com.y(2) = solution.com_act.acc.y_vec[next_sample];

	// TODO: Temporary solutions
	com.yaw(0) = solution.state_vec[0].trunkYaw_(next_sample);
	com.yaw(1) = solution.state_vec[1].trunkYaw_(next_sample);
	com.yaw(2) = solution.state_vec[2].trunkYaw_(next_sample);
	com.z(0) = data_robot_.com(2);
	com.z(1) = 0.0;
	com.z(2) = 0.0;

	com_->state(com);
}

void RigidBodySystem::setSelectionNumber(double firstSamplingPeriod){
	com_->setSelectionNumber(firstSamplingPeriod);
	left_foot_->setSelectionNumber(firstSamplingPeriod);
	right_foot_->setSelectionNumber(firstSamplingPeriod);
}

RigidBody *RigidBodySystem::body(BodyType type){
	switch(type){
	case COM:
		return com_;
	case LEFT_FOOT:
		return left_foot_;
	default:
		return right_foot_;
	}
}

const RigidBody *RigidBodySystem::body(BodyType type) const{
	switch(type){
	case COM:
		return com_;
	case LEFT_FOOT:
		return left_foot_;
	default:
		return right_foot_;
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
