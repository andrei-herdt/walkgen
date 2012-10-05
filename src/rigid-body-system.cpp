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
	if (com_ != 0x0) {
		delete com_;
		com_ = NULL;
	}

	if (left_foot_ != 0x0)
		delete left_foot_;

	if (right_foot_ != 0x0)
		delete right_foot_;
}

void RigidBodySystem::Init(const MPCParameters *mpc_parameters_p) {
	mpc_parameters_p_ = mpc_parameters_p;

	com_->Init(mpc_parameters_p_);
	com_->Init(&dynamics_builder_);
	left_foot_->Init(mpc_parameters_p_);
	right_foot_->Init(mpc_parameters_p_);
}

void RigidBodySystem::Init(const RobotData &robot_data) {//TODO: Remove object robot_data
	robot_data_ = robot_data;

	com_->Init(&robot_data);
	left_foot_->Init(&robot_data);
	right_foot_->Init(&robot_data);

	com_->state().x(0) = robot_data_.com(0);
	com_->state().y(0) = robot_data_.com(1);
	com_->state().z(0) = robot_data_.com(2);

	currentSupport_.phase         = DS;
	currentSupport_.foot          = LEFT;
	currentSupport_.time_limit    = 1e9;
	currentSupport_.num_steps_left   = 1;
	currentSupport_.state_changed = true;
	currentSupport_.x             = robot_data.leftFoot.position[0];
	currentSupport_.y             = robot_data.leftFoot.position[1];
	currentSupport_.yaw           = 0.0;
	currentSupport_.start_time    = 0.0;
}

void RigidBodySystem::ComputeDynamics() {
	com_->ComputeDynamics(mpc_parameters_p_->dynamics_order);
}

void RigidBodySystem::Interpolate(MPCSolution &solution, double current_time, const Reference &ref){
	com_->Interpolate(solution, current_time, ref);
	left_foot_->Interpolate(solution, current_time, ref);
	right_foot_->Interpolate(solution, current_time, ref);
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

	// TODO: Not necessary if feedback == true
	com_->state().x(0) = solution.com_act.pos.x_vec[next_sample];
	com_->state().y(0) = solution.com_act.pos.y_vec[next_sample];
	com_->state().x(1) = solution.com_act.vel.x_vec[next_sample];
	com_->state().y(1) = solution.com_act.vel.y_vec[next_sample];
	com_->state().x(2) = solution.com_act.acc.x_vec[next_sample];
	com_->state().y(2) = solution.com_act.acc.y_vec[next_sample];

	// TODO: Temporary solutions
	com_->state().yaw(0) = solution.com_act.pos.yaw_vec(next_sample);
	com_->state().yaw(1) = solution.com_act.vel.yaw_vec(next_sample);
	com_->state().yaw(2) = solution.com_act.acc.yaw_vec(next_sample);
	com_->state().z(0) = robot_data_.com(2);
}

void RigidBodySystem::setSelectionNumber(double firstSamplingPeriod){
	com_->SetSelectionNumber(firstSamplingPeriod);
	left_foot_->SetSelectionNumber(firstSamplingPeriod);
	right_foot_->SetSelectionNumber(firstSamplingPeriod);
}

RigidBody *RigidBodySystem::body(BodyType type){//TODO:Remove this
	switch(type){
	case COM:
		return com_;
	case LEFT_FOOT:
		return left_foot_;
	default:
		return right_foot_;
	}
}

const RigidBody *RigidBodySystem::body(BodyType type) const{//TODO: Remove this
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
	case FOOT_HULL:
		if (prwSupport.foot == LEFT){
			hull = robot_data_.left_foot_pos_hull;
		}else{
			hull = robot_data_.right_foot_pos_hull;
		}
		break;
	case COP_HULL:
		if (prwSupport.foot == LEFT){
			if (prwSupport.phase == SS){
				hull = robot_data_.left_foot_ss_hull;
			}else{
				hull = robot_data_.left_foot_ds_hull;
			}
		}else{
			if (prwSupport.phase == SS){
				hull = robot_data_.right_foot_ss_hull;
			}else{
				hull =  robot_data_.right_foot_ds_hull;
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
