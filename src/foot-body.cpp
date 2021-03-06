#include <mpc-walkgen/foot-body.h>
#include <mpc-walkgen/tools.h>
#include <mpc-walkgen/debug.h>

#include <iostream>

using namespace MPCWalkgen;
using namespace Eigen;

//
// Public methods:
//
FootBody::FootBody(Foot which) : RigidBody()
,which_(which) {
}

FootBody::~FootBody(){}

void FootBody::Interpolate(MPCSolution &solution, double current_time, const Reference &/*ref*/) {
	assert(mpc_parameters_->period_ss > kEps);

	BodyState goal_state;
	const SupportState &current_support = solution.support_states_vec[0];
	const SupportState &next_support = solution.support_states_vec[1];
	int num_steps_previewed = solution.support_states_vec.back().step_number;
	int num_unst_modes = 0;
	if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		num_unst_modes = 1;
	}

	double time_left_xy = 1; // Duration of the current interpolation phase of the horizontal motion
	double time_left_z = 1; // Duration of the current interpolation phase of the vertical motion
	int num_samples = mpc_parameters_->num_samples_act();
	double period_ds = mpc_parameters_->period_trans_ds();
	double raise_period = std::max(0.05, mpc_parameters_->period_recomputation); // Time during which the horizontal displacement is blocked
	double time_until_td = 0.0;  // Time left until touchdown
	double time_spent_flying = 0.0;
	double sec_margin = 0.1;// New
	double halftime_rounded = (mpc_parameters_->period_ss - sec_margin) / 2.;
	if (current_support.phase == SS) {
		time_until_td = current_support.time_limit - current_time;
		time_spent_flying = current_time - current_support.start_time - period_ds - sec_margin;
		if (next_support.transitional_ds) {//Don't move
			time_left_xy = 1.;//current_support.time_limit - current_time;
			goal_state.x(0) = state_.x(0);
			goal_state.y(0) = state_.y(0);
			goal_state.yaw(0) = state_.yaw(0);
		} else if (time_spent_flying < raise_period - kEps) {//Don't move
			time_left_xy = 1.;//raise_period - time_spent_flying;
			goal_state.x(0) = state_.x(0);
			goal_state.y(0) = state_.y(0);
			goal_state.yaw(0) = state_.yaw(0);
		}  else if (time_until_td < raise_period + kEps) {//Don't move
			time_left_xy = 1.;
			goal_state.x(0) = state_.x(0);
			goal_state.y(0) = state_.y(0);
			goal_state.yaw(0) = state_.yaw(0);
		} else {//Move
			time_left_xy = time_until_td - raise_period;
			if (num_steps_previewed > 0) {
				goal_state.x(0) = solution.qp_solution_vec(2*(mpc_parameters_->num_samples_contr + num_unst_modes));
				goal_state.y(0) = solution.qp_solution_vec(2*(mpc_parameters_->num_samples_contr + num_unst_modes) + num_steps_previewed);
				goal_state.yaw(0) = solution.support_yaw_vec[0];
			} else {
				goal_state.x(0) = state_.x(0);
				goal_state.y(0) = state_.y(0);
				goal_state.yaw(0) = state_.yaw(0);
			}
		}
		// Vertical trajectory
		if (time_until_td - halftime_rounded > mpc_parameters_->period_actsample && time_spent_flying > kEps) {
			goal_state.z(0) = robot_data_->max_foot_height;
			time_left_z = time_until_td - halftime_rounded;
		} else if (time_until_td < halftime_rounded && time_until_td > kEps) { // Half-time passed
			time_left_z = time_until_td;
		} else {
			// time_left_z stays 1
		}
		InterpolatePolynomial(solution, motion_act_.pos.x_vec, motion_act_.vel.x_vec, motion_act_.acc.x_vec, num_samples, time_left_xy, state_.x, goal_state.x);
		InterpolatePolynomial(solution, motion_act_.pos.y_vec, motion_act_.vel.y_vec, motion_act_.acc.y_vec, num_samples, time_left_xy, state_.y, goal_state.y);
		InterpolatePolynomial(solution, motion_act_.pos.z_vec, motion_act_.vel.z_vec, motion_act_.acc.z_vec, num_samples, time_left_z, state_.z, goal_state.z);
		InterpolatePolynomial(solution, motion_act_.pos.yaw_vec, motion_act_.vel.yaw_vec, motion_act_.acc.yaw_vec, num_samples, time_left_xy, state_.yaw, goal_state.yaw);
	} else {
		InterpolatePolynomial(solution, motion_act_.pos.x_vec, motion_act_.vel.x_vec, motion_act_.acc.x_vec, num_samples, time_left_xy, state_.x, state_.x);
		InterpolatePolynomial(solution, motion_act_.pos.y_vec, motion_act_.vel.y_vec, motion_act_.acc.y_vec, num_samples, time_left_xy, state_.y, state_.y);
		InterpolatePolynomial(solution, motion_act_.pos.z_vec, motion_act_.vel.z_vec, motion_act_.acc.z_vec, num_samples, time_left_z, state_.z, state_.z);
		InterpolatePolynomial(solution, motion_act_.pos.yaw_vec, motion_act_.vel.yaw_vec, motion_act_.acc.yaw_vec, num_samples, time_left_xy, state_.yaw, state_.yaw);
	}

}


//
// Private methods:
//
void FootBody::InterpolatePolynomial(MPCSolution &solution, CommonVectorType &pos_vec, CommonVectorType &vel_vec, CommonVectorType &acc_vec, 
		int num_samples, double T, const Eigen::Vector3d &init_state_vec, const Eigen::Vector3d &final_state_vec) {
	if (pos_vec.rows() < num_samples) {
		pos_vec.resize(num_samples);
		vel_vec.resize(num_samples);
		acc_vec.resize(num_samples);
	}

	if (solution.support_states_vec[0].foot == which_) {
		pos_vec.setConstant(init_state_vec[0]);
		vel_vec.setConstant(0.0);
		acc_vec.setConstant(0.0);
	} else {
		Eigen::Matrix<double, 6, 1> factors;
		interpolation_.ComputeNormalPolynomCoefficients(factors, init_state_vec, final_state_vec, T);
		for (int i = 0; i < num_samples; ++i) {
			double ti = (i + 1) * mpc_parameters_->period_actsample;

			pos_vec[i] = p(factors, ti / T);
			vel_vec[i] = dp(factors, ti / T) / T;
			acc_vec[i] = ddp(factors, ti / T) / (T * T);
		}
	}
}
