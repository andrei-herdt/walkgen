#include <mpc-walkgen/rigid-body.h>
#include <mpc-walkgen/debug.h>

using namespace MPCWalkgen;


RigidBody::RigidBody():mpc_parameters_(NULL)
,robot_data_(NULL)
,dyn_build_p_(NULL)
{}

RigidBody::~RigidBody() {}

void RigidBody::Init(const MPCParameters *mpc_parameters_p) {
	mpc_parameters_ = mpc_parameters_p;

	int num_dynamics = mpc_parameters_->GetNumDynamics();
	dynamics_qp_vec_.resize(num_dynamics);

	int num_samples = mpc_parameters_->num_samples_act();
	int num_unst_modes = 1;//TODO: unstable modes
	motion_act_.SetZero(num_samples, num_unst_modes);
	wrench_.SetZero();
}

void RigidBody::Init(const RobotData *robot_data_p) {
	robot_data_ = robot_data_p;
}

void RigidBody::Init(DynamicsBuilder *dyn_build_p) {
	dyn_build_p_ = dyn_build_p;
}

void RigidBody::ComputeDynamics(SystemOrder dynamics_order) {
	assert(state_.z(0) > kEps);
	assert(mpc_parameters_->num_samples_horizon_max > 0);

	int num_samples = mpc_parameters_->num_samples_horizon_max;
	int num_dynamics = mpc_parameters_->GetNumDynamics();

	if (mpc_parameters_->mapping == ZERO_MAP) {
		std::vector<double> st_sampling_periods_vec(num_samples, mpc_parameters_->period_qpsample);
		std::vector<double> inp_sampling_periods_vec(num_samples, mpc_parameters_->period_qpsample);
		if (mpc_parameters_->num_samples_first_fine_period > 0 || mpc_parameters_->num_samples_first_coarse_period > 0) {
			// Build vectors of sampling periods:
			// ----------------------------------
			int period_num = 0;
			for(; period_num < mpc_parameters_->num_samples_first_fine_period; period_num++) {
				st_sampling_periods_vec[period_num] = mpc_parameters_->period_recomputation;
			}
			for(; period_num < mpc_parameters_->num_samples_first_coarse_period + mpc_parameters_->num_samples_first_fine_period - 1; period_num++) {
				st_sampling_periods_vec[period_num] = mpc_parameters_->period_inter_samples;
			}

			int num_cycles_ext = static_cast<int>(mpc_parameters_->period_qpsample / mpc_parameters_->period_inter_samples);
			int num_cycles_int = static_cast<int>(mpc_parameters_->period_inter_samples / mpc_parameters_->period_recomputation);

			std::vector<LinearDynamics>::iterator dyn_it = dynamics_qp_vec_.begin();
			double last_period = 0.;
			double first_coarse_period = mpc_parameters_->period_qpsample;
			for (int k = 0; k < num_cycles_ext; ++k) {
				double first_fine_period = mpc_parameters_->period_inter_samples;
				for (int i = 0; i < num_cycles_int; ++i) {
					st_sampling_periods_vec.back() = last_period;
					st_sampling_periods_vec.at(mpc_parameters_->num_samples_first_fine_period) = first_fine_period;
					st_sampling_periods_vec.at(mpc_parameters_->num_samples_first_fine_period + mpc_parameters_->num_samples_first_coarse_period - 1) = first_coarse_period;
					if (st_sampling_periods_vec.back() < kEps) {
						st_sampling_periods_vec.pop_back();
						dyn_build_p_->Build(dynamics_order, *dyn_it, state_.z(0), st_sampling_periods_vec, st_sampling_periods_vec, num_samples, false);
						st_sampling_periods_vec.push_back(0.);
					} else {
						dyn_build_p_->Build(dynamics_order, *dyn_it, state_.z(0), st_sampling_periods_vec, st_sampling_periods_vec, num_samples, false);
					}
					++dyn_it;
					first_fine_period -= mpc_parameters_->period_recomputation;
					last_period += mpc_parameters_->period_recomputation;
				}
				first_coarse_period -= mpc_parameters_->period_inter_samples;
			}
		} else {
			// Build vectors of sampling periods:
			// ----------------------------------
			std::vector<double> st_sampling_periods_vec(num_samples, mpc_parameters_->period_qpsample);
			std::vector<LinearDynamics>::iterator dyn_it = dynamics_qp_vec_.begin();
			for (int k = 0; k < num_dynamics; ++k) {
				st_sampling_periods_vec[0] = mpc_parameters_->period_qpsample - mpc_parameters_->period_recomputation * k;
				std::copy(inp_sampling_periods_vec.begin(), inp_sampling_periods_vec.end(), st_sampling_periods_vec.begin());
				dyn_build_p_->Build(dynamics_order, *dyn_it, state_.z(0), st_sampling_periods_vec, inp_sampling_periods_vec, num_samples, false);
				++dyn_it;
			}
		}

	} else if (mpc_parameters_->mapping == CONST_MAP) {
		int ns_st = mpc_parameters_->num_samples_state;
		int ns_c = mpc_parameters_->num_samples_contr;

		// Generate sampling vectors:
		// --------------------------
		std::vector<double> st_sampling_periods_vec(ns_st, mpc_parameters_->period_actsample);

		//std::vector<double> inp_sampling_periods_vec(ns_c, mpc_parameters_->period_qpsample / 2.);

		std::vector<double> inp_sampling_periods_vec(ns_c, mpc_parameters_->period_qpsample);
		int period_num = 0;
		for(; period_num < mpc_parameters_->num_samples_first_fine_period; period_num++) {
			inp_sampling_periods_vec.at(period_num) = mpc_parameters_->period_recomputation;
		}
		for(; period_num < mpc_parameters_->num_samples_first_coarse_period + mpc_parameters_->num_samples_first_fine_period - 1; period_num++) {
			inp_sampling_periods_vec.at(period_num) = mpc_parameters_->period_inter_samples;
		}

		// Generate dynamics:
		// ------------------
		dyn_build_p_->Build(dynamics_order, dynamics_qp_vec_.front(), state_.z(0), st_sampling_periods_vec, inp_sampling_periods_vec, ns_c, false);
	}

	num_samples = mpc_parameters_->num_samples_act();
	std::vector<double> st_sampling_periods_vec(num_samples, mpc_parameters_->period_actsample);
	std::vector<double> inp_sampling_periods_vec(num_samples, mpc_parameters_->period_actsample);
	dyn_build_p_->Build(dynamics_order, dynamics_act_, state_.z(0), st_sampling_periods_vec, inp_sampling_periods_vec, num_samples, true);
}

//
// Tests:
//
void RigidBody::CPOutput2Control(CommonVectorType &contr_vec, const CommonVectorType &output_vec) {
	assert(output_vec.rows() == dynamics_qp_vec_.front().cp.input_mat.rows());

	const LinearDynamicsMatrices &pos_dyn_mat = dynamics_qp_vec_.front().pos;
	int num_cols = pos_dyn_mat.input_mat.cols();
	int num_rows = pos_dyn_mat.input_mat.rows();
	const LinearDynamicsMatrices &vel_dyn_mat = dynamics_qp_vec_.front().vel;

	double omega = sqrt(kGravity / state_.z[0]);
	CommonMatrixType input_mat = pos_dyn_mat.input_mat.block(0, 0, num_rows, num_cols - 1)
											+ 1. / omega * vel_dyn_mat.input_mat.block(0, 0, num_rows, num_cols - 1);
	CommonMatrixType unst_state_mat = pos_dyn_mat.input_mat.block(0, num_cols - 1, num_rows, 1)
												+ 1. / omega * vel_dyn_mat.input_mat.block(0, num_cols - 1, num_rows, 1);
	CommonMatrixType stab_state_mat = pos_dyn_mat.state_mat
			+ 1. / omega * vel_dyn_mat.state_mat;

	if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		Matrix2D state_trans_mat = Matrix2D::Zero();
		state_trans_mat(0, 0) = 1.;
		state_trans_mat(0, 1) = -1. / omega;
		state_trans_mat(1, 0) = 1.;
		state_trans_mat(1, 1) = 1. / omega;
		CommonVectorType new_state = state_trans_mat * state_.x.head(2);
		CommonVectorType state = new_state.block(0, 0, 1, 1);

		CommonVectorType tmp_vec = unst_state_mat * output_vec.segment(num_rows - 1, 1);
		contr_vec = input_mat.inverse() * (output_vec - stab_state_mat * state - tmp_vec);
	}
}
