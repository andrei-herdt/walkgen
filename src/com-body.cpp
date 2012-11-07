#include <mpc-walkgen/com-body.h>
#include <mpc-walkgen/tools.h>

#include <iostream>

using namespace MPCWalkgen;
using namespace Eigen;


CoMBody::CoMBody() : RigidBody() {}

CoMBody::~CoMBody() {}

void CoMBody::Interpolate(MPCSolution &solution, double current_time, const Reference &ref){
	// Actuator sampling rate:
	// -----------------------
	CommonVectorType state_x(mpc_parameters_->dynamics_order), state_y(mpc_parameters_->dynamics_order);
	if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		Matrix2D state_trans_mat;
		state_trans_mat.setZero();
		state_trans_mat(0, 0) = 1.; state_trans_mat(0, 1) = -1./sqrt(kGravity / state_.z[0]);
		state_trans_mat(1, 0) = 1.; state_trans_mat(1, 1) = 1./sqrt(kGravity / state_.z[0]);
		state_x = state_trans_mat * state_.x.head(mpc_parameters_->dynamics_order);
		state_y = state_trans_mat * state_.y.head(mpc_parameters_->dynamics_order);
	} else {
		for (int i = 0; i < mpc_parameters_->dynamics_order; i++) {
			state_x(i) = state_.x(i);
			state_y(i) = state_.y(i);
		}
	}

	// Position:
	interpolation_.Interpolate(solution.com_act.pos.x_vec, dynamics_act().pos,
			state_x, solution.com_prw.control.x_vec[0]);
	interpolation_.Interpolate(solution.com_act.pos.y_vec, dynamics_act().pos,
			state_y, solution.com_prw.control.y_vec[0]);

	// Velocity:
	interpolation_.Interpolate(solution.com_act.vel.x_vec, dynamics_act().vel,
			state_x, solution.com_prw.control.x_vec[0]);
	interpolation_.Interpolate(solution.com_act.vel.y_vec, dynamics_act().vel,
			state_y, solution.com_prw.control.y_vec[0]);

	// Acceleration:
	interpolation_.Interpolate(solution.com_act.acc.x_vec, dynamics_act().acc,
			state_x, solution.com_prw.control.x_vec[0]);
	interpolation_.Interpolate(solution.com_act.acc.y_vec, dynamics_act().acc,
			state_y, solution.com_prw.control.y_vec[0]);

	// CoP:
	interpolation_.Interpolate(solution.com_act.cop.x_vec, dynamics_act().cop,
			state_x, solution.com_prw.control.x_vec[0]);
	interpolation_.Interpolate(solution.com_act.cop.y_vec, dynamics_act().cop,
			state_y, solution.com_prw.control.y_vec[0]);

	// QP sampling rate:
	// -----------------
	// Position:
	if (mpc_parameters_->interpolate_whole_horizon) {
		int samples_left = mpc_parameters_->GetMPCSamplesLeft(solution.sampling_times_vec[1] - solution.sampling_times_vec[0]);
		interpolation_.Interpolate(solution.com_prw.pos.x_vec, dynamics_qp()[samples_left].pos,
				state_x, solution.com_prw.control.x_vec);
		interpolation_.Interpolate(solution.com_prw.pos.y_vec, dynamics_qp()[samples_left].pos,
				state_y, solution.com_prw.control.y_vec);

		// Velocity:
		interpolation_.Interpolate(solution.com_prw.vel.x_vec, dynamics_qp()[samples_left].vel,
				state_x, solution.com_prw.control.x_vec);
		interpolation_.Interpolate(solution.com_prw.vel.y_vec, dynamics_qp()[samples_left].vel,
				state_y, solution.com_prw.control.y_vec);

		// Acceleration:
		interpolation_.Interpolate(solution.com_prw.acc.x_vec, dynamics_qp()[samples_left].acc,
				state_x, solution.com_prw.control.x_vec);
		interpolation_.Interpolate(solution.com_prw.acc.y_vec, dynamics_qp()[samples_left].acc,
				state_y, solution.com_prw.control.y_vec);

		// Capture point:
		interpolation_.Interpolate(solution.com_prw.cp.x_vec, dynamics_qp()[samples_left].cp,
				state_x, solution.com_prw.control.x_vec);
		interpolation_.Interpolate(solution.com_prw.cp.y_vec, dynamics_qp()[samples_left].cp,
				state_y, solution.com_prw.control.y_vec);

		// Center of pressure:
		interpolation_.Interpolate(solution.com_prw.cop.x_vec, dynamics_qp()[samples_left].cop,
				state_x, solution.com_prw.control.x_vec);
		interpolation_.Interpolate(solution.com_prw.cop.y_vec, dynamics_qp()[samples_left].cop,
				state_y, solution.com_prw.control.y_vec);
	}

	InterpolateTrunkYaw(solution, ref);
}

void CoMBody::InterpolateTrunkYaw(MPCSolution &solution, const Reference &ref) {
	double T = mpc_parameters_->num_samples_step * mpc_parameters_->period_qpsample;

	int num_samples = mpc_parameters_->num_samples_act();

	if (solution.com_act.pos.yaw_vec.rows() != num_samples){
		solution.com_act.pos.yaw_vec.resize(num_samples);
		solution.com_act.vel.yaw_vec.resize(num_samples);
		solution.com_act.acc.yaw_vec.resize(num_samples);
	}

	double orientation0 = solution.trunk_yaw_vec[0];
	double orientation1;

	int size = solution.trunk_yaw_vec.size();
	if (size >= 2) {
		orientation1 = solution.trunk_yaw_vec[1];
	} else {
		orientation1 = orientation0;
	}

	Vector3d nextTrunkState(orientation1, -ref.local.yaw(0), 0);

	Eigen::Matrix<double,6,1> factor;
	interpolation_.ComputeNormalPolynomCoefficients(factor, state().yaw, nextTrunkState, T);
	for (int i=0; i < num_samples; ++i) {
		double ti = (i + 1) * mpc_parameters_->period_actsample;

		solution.com_act.pos.yaw_vec(i) = p(factor, ti/T);
		solution.com_act.vel.yaw_vec(i) = dp(factor, ti/T)/T;
		solution.com_act.acc.yaw_vec(i) = ddp(factor, ti/T)/(T*T);
	}


}
