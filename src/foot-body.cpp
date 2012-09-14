#include <mpc-walkgen/foot-body.h>
#include <mpc-walkgen/tools.h>

using namespace MPCWalkgen;
using namespace Eigen;

//
// Public methods:
//
FootBody::FootBody(Foot which) : RigidBody()
,which_(which) {
}

FootBody::~FootBody(){}

void FootBody::ComputeDynamicsMatrices(LinearDynamicsMatrices &dyn,
									   double sample_period_first, double sample_period_rest, int nbsamples, Derivative type) {
}

void FootBody::Interpolate(MPCSolution &solution, double currentTime, const Reference &/*velRef*/) {
	BodyState goal_state;
	const SupportState &current_support = solution.support_states_vec[0];
	const SupportState &next_support = solution.support_states_vec[1];

	double time_left_xy = 1; // Duration of the current interpolation phase of the horizontal motion
	double time_left_z = 1; // Duration of the current interpolation phase of the vertical motion
	int num_samples = mpc_parameters_->num_samples_act();
	double period_ds = mpc_parameters_->period_trans_ds();
	double raise_period = std::max(0.05, mpc_parameters_->period_mpcsample); // Time during which the horizontal displacement is blocked
	double time_left_flying = 0.0;
	double time_spent_flying = 0.0;
	double halftime_rounded = std::ceil(static_cast<double>(mpc_parameters_->num_qpsamples_ss()) / 2.0) * mpc_parameters_->period_qpsample;
	if (current_support.phase == SS) {
		time_left_flying = current_support.time_limit - period_ds - currentTime;
		time_spent_flying = currentTime - current_support.start_time;
		int nbStepsPreviewed = solution.support_states_vec.back().stepNumber;
		if (next_support.transitional_ds) {
			time_left_xy = current_support.time_limit - currentTime;
			goal_state.x(0) = state_.x(0);
			goal_state.y(0) = state_.y(0);
			goal_state.yaw(0) = state_.yaw(0);
		} else if (time_spent_flying < raise_period - EPSILON) {
			time_left_xy = raise_period - time_spent_flying;
			goal_state.x(0) = state_.x(0);
			goal_state.y(0) = state_.y(0);
			goal_state.yaw(0) = state_.yaw(0);
		}  else if (time_left_flying < raise_period + EPSILON) {
			time_left_xy = time_left_flying;
			goal_state.x(0) = state_.x(0);
			goal_state.y(0) = state_.y(0);
			goal_state.yaw(0) = state_.yaw(0);
		} else {
			time_left_xy = time_left_flying - raise_period;
			int nbPreviewedSteps = solution.support_states_vec.back().stepNumber;
			if (nbPreviewedSteps > 0) {
				goal_state.x(0) = solution.qpSolution(2 * mpc_parameters_->nbsamples_qp);
				goal_state.y(0) = solution.qpSolution(2 * mpc_parameters_->nbsamples_qp + nbStepsPreviewed);
				goal_state.yaw(0) = solution.supportOrientations_vec[0];
			} else {
				goal_state.x(0) = state_.x(0);
				goal_state.y(0) = state_.y(0);
				goal_state.yaw(0) = state_.yaw(0);
			}
		}
		// Vertical trajectory
		if (time_left_flying - halftime_rounded > mpc_parameters_->period_actsample) {
			goal_state.z(0) = robot_data_p_->freeFlyingFootMaxHeight;
			time_left_z = time_left_flying - halftime_rounded;
		} else if (time_left_flying < halftime_rounded && time_left_flying > EPSILON) { // Half-time passed
			time_left_z = time_left_flying;
		} else {
			// time_left_z stays 1
		}
	} 
		InterpolatePolynomial(solution, motion_act_.pos.x_vec, motion_act_.vel.x_vec, motion_act_.acc.x_vec, num_samples, time_left_xy, state_.x, goal_state.x);
		InterpolatePolynomial(solution, motion_act_.pos.y_vec, motion_act_.vel.y_vec, motion_act_.acc.y_vec, num_samples, time_left_xy, state_.x, goal_state.x);
		InterpolatePolynomial(solution, motion_act_.pos.z_vec, motion_act_.vel.z_vec, motion_act_.acc.z_vec, num_samples, time_left_xy, state_.x, goal_state.x);
		InterpolatePolynomial(solution, motion_act_.pos.yaw_vec, motion_act_.vel.yaw_vec, motion_act_.acc.yaw_vec, num_samples, time_left_xy, state_.x, goal_state.x);
}


//
// Private methods:
//
/*
VectorXd &FootBody::SetTrajectoryPointer() {
	if (which_ == LEFT) {
		Motion *trajectory_ = &solution.left_foot_acc;
	} else {
		Motion *trajectory_ = &solution.right_foot_acc;
	}
}
*/

void FootBody::InterpolatePolynomial(MPCSolution &solution, CommonVectorType &pos_vec, CommonVectorType &vel_vec, CommonVectorType &acc_vec, 
									 int num_samples, double T, const Eigen::Vector3d &init_state_vec, const Eigen::Vector3d &final_state_vec) {
	if (pos_vec.rows() < num_samples) {
		pos_vec.resize(num_samples);
		vel_vec.resize(num_samples);
		acc_vec.resize(num_samples);
	}

	if (solution.support_states_vec[0].foot == which_) {
		pos_vec.fill(init_state_vec[0]);
		vel_vec.fill(0.0);
		acc_vec.fill(0.0);
	} else {
		if (solution.support_states_vec[0].phase == DS) {
			pos_vec.fill(init_state_vec[0]);
			vel_vec.fill(0.0);
			acc_vec.fill(0.0);
		} else {
			Eigen::Matrix<double, 6, 1> factors;
			interpolation_.computePolynomialNormalisedFactors(factors, init_state_vec, final_state_vec, T);
			for (int i = 0; i < num_samples; ++i) {
				double ti = (i + 1) * mpc_parameters_->period_actsample;

				pos_vec[i] = p(factors, ti / T);
				vel_vec[i]   = dp(factors, ti / T) / T;
				acc_vec[i]   = ddp(factors, ti / T) / (T * T);
			}
		}
	}
}


/*
void FootBody::InterpolatePolynomial(MPCSolution &solution, Axis axis, int num_samples, double T, 
									 const Eigen::Vector3d &init_state_vec, const Eigen::Vector3d &final_state_vec) {
	VectorXd &FootTrajState = getFootVector(solution, axis, POSITION);
	VectorXd &FootTrajVel   = getFootVector(solution, axis, 1);
	VectorXd &FootTrajAcc   = getFootVector(solution, axis, 2);

	if (FootTrajState.rows() != num_samples) {
		FootTrajState.resize(num_samples);
		FootTrajVel.resize(num_samples);
		FootTrajAcc.resize(num_samples);
	}

	if (solution.support_states_vec[0].foot == which_) {
		FootTrajState.fill(init_state_vec(0));
		FootTrajVel.fill(0.0);
		FootTrajAcc.fill(0.0)
	} else {
		if (solution.support_states_vec[0].phase == DS) {
			FootTrajState.fill(init_state_vec(0));
			FootTrajVel.fill(0.0);
			FootTrajAcc.fill(0.0);

		} else {
			Eigen::Matrix<double,6,1> factors;
			interpolation_.computePolynomialNormalisedFactors(factors, init_state_vec, final_state_vec, T);
			for (int i = 0; i < num_samples; ++i) {
				double ti = (i+1)*mpc_parameters_->period_actsample;

				FootTrajState(i) = p(factors, ti/T);
				FootTrajVel(i)   = dp(factors, ti/T)/T;
				FootTrajAcc(i)   = ddp(factors, ti/T)/(T*T);
			}
		}
	}
}
*/