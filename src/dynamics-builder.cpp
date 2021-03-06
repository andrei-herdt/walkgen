#include <mpc-walkgen/dynamics-builder.h>
#include <mpc-walkgen/tools.h>
#include <mpc-walkgen/debug.h>

#include <cmath>

using namespace MPCWalkgen;

//
// Public methods:
//
DynamicsBuilder::DynamicsBuilder():mpc_parameters_(NULL){}
//TODO: ,state_mat_vec(std::vector<Matrix2D>(mpc_parameters->num_samples_horizon,Matrix2D::Zero()))
// ,input_mat_vec(std::vector<Vector2D>(mpc_parameters->num_samples_horizon,Vector2D::Zero())){


DynamicsBuilder::~DynamicsBuilder() {}

void DynamicsBuilder::Init(const MPCParameters *mpc_parameters) {
	mpc_parameters_ = mpc_parameters;

	eigenval_vec_.setZero(); eigenvec_mat_.setZero(); eigenvec_mat_inv_.setZero();
	diag_exp_eig_mat_.setZero();

	identity_mat_.setIdentity();

	precomp_input_mat_.setZero();  // \f[ -A^{-1}\mathbb{I}B \f]
}

void DynamicsBuilder::Build(SystemOrder dynamics_order, LinearDynamics &dyn, double height, const std::vector<double> &st_sampling_periods_vec,
		const std::vector<double> &inp_sampling_periods_vec, int num_samples, bool actuation) {
	switch (dynamics_order) {
	case FIRST_ORDER:
		break;
	case SECOND_ORDER:
		BuildSecondOrder(dyn, height, st_sampling_periods_vec, inp_sampling_periods_vec, actuation);
		break;
	case THIRD_ORDER:
		BuildThirdOrder(dyn, height, st_sampling_periods_vec, num_samples);
		break;
	}
}

//
// Private methods:
//
void DynamicsBuilder::BuildSecondOrder(LinearDynamics &dyn, double height, const std::vector<double> &st_sampling_periods_vec,
		const std::vector<double> &inp_sampling_periods_vec, bool is_actuation) {
	assert(st_sampling_periods_vec.at(0) > 0.); assert(inp_sampling_periods_vec.at(0) > 0.);
	assert(st_sampling_periods_vec.size() > 0); assert(inp_sampling_periods_vec.size() > 0);

	/*
	BuildSecondOrderCoPOutput(dyn.pos, height, sample_period_first, sample_period_rest, num_samples, POSITION);
	BuildSecondOrderCoPOutput(dyn.vel, height, sample_period_first, sample_period_rest, num_samples, VELOCITY);
	BuildSecondOrderCoPOutput(dyn.acc, height, sample_period_first, sample_period_rest, num_samples, ACCELERATION);
	BuildSecondOrderCoPOutput(dyn.cop, height, sample_period_first, sample_period_rest, num_samples, COP);
	 */

	int num_samples_state = static_cast<int>(st_sampling_periods_vec.size());
	int num_samples_contr = static_cast<int>(inp_sampling_periods_vec.size());
	double omega = sqrt(kGravity / height);
	double omega_square = kGravity / height;
	if (mpc_parameters_->formulation == STANDARD || is_actuation) {
		double sample_period_first = st_sampling_periods_vec.at(0); // TODO: Remove dependency on sp_first, sp_rest
		double sample_period_rest = sample_period_first;
		if (st_sampling_periods_vec.size() > 1) {
			sample_period_rest = st_sampling_periods_vec.at(1);
		}

		int state_dim = 2;
		int num_stable_modes = state_dim;
		int num_unstable_modes = state_dim - num_stable_modes;
		int input_dim = 1;
		int output_dim = 1;
		dyn.SetZero(state_dim, input_dim, output_dim, num_samples_state, num_samples_contr, num_stable_modes, num_unstable_modes);

		// Continuous (general) state space dynamics:
		// ------------------------------------------

		dyn.cont_ss.c_state_mat(0, 1) = 1.;
		dyn.cont_ss.c_state_mat(1, 0) = omega_square;
		dyn.cont_ss.c_state_mat_inv = dyn.cont_ss.c_state_mat.inverse();

		dyn.cont_ss.c_input_mat(0) = 0.;
		dyn.cont_ss.c_input_mat(1) = - omega_square;

		dyn.pos.ss_output_mat(0, 0) = 1.;
		dyn.vel.ss_output_mat(0, 1) = 1.;
		dyn.cp.ss_output_mat(0, 0) = 1.;
		dyn.cp.ss_output_mat(0, 1) = 1./omega;

		/*
		dyn.cont_ss.c_state_mat(0, 0) = -omega;
		dyn.cont_ss.c_state_mat(1, 1) = omega;
		dyn.cont_ss.c_state_mat_inv = dyn.cont_ss.c_state_mat.inverse();
		dyn.cont_ss.c_input_mat(0) = omega;
		dyn.cont_ss.c_input_mat(1) = -omega;

		dyn.pos.ss_output_mat(0, 0) = 1./2.;
		dyn.pos.ss_output_mat(0, 1) = 1./2.;
		dyn.vel.ss_output_mat(0, 0) = -omega/2.;
		dyn.vel.ss_output_mat(0, 1) = omega/2.;
		dyn.cp.ss_output_mat(0, 0) = 0.;
		dyn.cp.ss_output_mat(0, 1) = 1.;
		 */

		dyn.pos.c_state_mat 	= dyn.cont_ss.c_state_mat;
		dyn.pos.c_state_mat_inv = dyn.cont_ss.c_state_mat_inv;
		dyn.pos.c_input_mat 	= dyn.cont_ss.c_input_mat;
		BuildSecondOrderCoPInputGeneral(dyn.pos, height, sample_period_first, sample_period_rest, num_samples_state);

		dyn.vel.c_state_mat 	= dyn.cont_ss.c_state_mat;
		dyn.vel.c_state_mat_inv = dyn.cont_ss.c_state_mat_inv;
		dyn.vel.c_input_mat 	= dyn.cont_ss.c_input_mat;
		BuildSecondOrderCoPInputGeneral(dyn.vel, height, sample_period_first, sample_period_rest, num_samples_state);

		dyn.cp.c_state_mat 		= dyn.cont_ss.c_state_mat;
		dyn.cp.c_state_mat_inv 	= dyn.cont_ss.c_state_mat_inv;
		dyn.cp.c_input_mat 		= dyn.cont_ss.c_input_mat;
		BuildSecondOrderCoPInputGeneral(dyn.cp, height, sample_period_first, sample_period_rest, num_samples_state);

		dyn.cop.input_mat.setIdentity();
		dyn.cop.input_mat_tr.setIdentity();
		dyn.cop.input_mat_inv.setIdentity();
		dyn.cop.input_mat_inv_tr.setIdentity();

		dyn.acc.state_mat = omega_square*(dyn.pos.state_mat - dyn.cop.state_mat);
		dyn.acc.input_mat = omega_square*(dyn.pos.input_mat - dyn.cop.input_mat);

	} else if (mpc_parameters_->formulation == DECOUPLED_MODES && !is_actuation) {
		int state_dim = 2;
		int num_stable_modes = 1;
		int num_unstable_modes = state_dim - num_stable_modes;
		int input_dim = 1;
		int output_dim = 1;
		dyn.SetZero(state_dim, input_dim, output_dim, num_samples_state, num_samples_contr, num_stable_modes, num_unstable_modes);

		// Continuous (general) state space dynamics:
		// ------------------------------------------
		dyn.cont_ss.c_state_mat(0, 0) 	= -omega;
		dyn.cont_ss.c_state_mat(1, 1) 	= omega;
		dyn.cont_ss.c_state_mat_inv 	= dyn.cont_ss.c_state_mat.inverse();
		dyn.cont_ss.c_input_mat(0) 		= omega;
		dyn.cont_ss.c_input_mat(1) 		= -omega;

		dyn.pos.ss_output_mat(0, 0) 	= 1./2.;
		dyn.pos.ss_output_mat(0, 1) 	= 1./2.;
		dyn.vel.ss_output_mat(0, 0) 	= -omega/2.;
		dyn.vel.ss_output_mat(0, 1) 	= omega/2.;
		dyn.cp.ss_output_mat(0, 0) 		= 0.;
		dyn.cp.ss_output_mat(0, 1) 		= 1.;

		// Build discrete state space dynamics:
		// ------------------------------------
		ComputeDiscreteSSDynamics(dyn, st_sampling_periods_vec);

		// Build products of state matrices:
		// ---------------------------------
		BuildStateMatrixProducts(dyn.d_state_mat_pow_vec, dyn.rev_matrix_prod_vec, dyn.d_state_mat_vec);

		// Build prediction matrices (S, U, UT, Uinv, UTinv):
		// --------------------------------------------------
		BuildSecondOrderCoPInputDecoupled(dyn.pos, dyn.d_state_mat_vec, dyn.d_input_mat_vec, dyn.d_state_mat_pow_vec, dyn.rev_matrix_prod_vec);
		BuildSecondOrderCoPInputDecoupled(dyn.vel, dyn.d_state_mat_vec, dyn.d_input_mat_vec, dyn.d_state_mat_pow_vec, dyn.rev_matrix_prod_vec);
		BuildSecondOrderCoPInputDecoupled(dyn.cp, dyn.d_state_mat_vec, dyn.d_input_mat_vec, dyn.d_state_mat_pow_vec, dyn.rev_matrix_prod_vec);

		dyn.cop.input_mat.block(0, 0, num_samples_state, num_samples_state).setIdentity();
		dyn.cop.input_mat_tr.block(0, 0, num_samples_state, num_samples_state).setIdentity();

		dyn.acc.state_mat = omega_square*(dyn.pos.state_mat - dyn.cop.state_mat);
		dyn.acc.input_mat = omega_square*(dyn.pos.input_mat - dyn.cop.input_mat);
	}

	// Build input map matrix:
	// -----------------------
	int num_unstable_modes = 0;
	if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		num_unstable_modes = 1;
	}
	CommonMatrixType input_map_mat = CommonMatrixType::Zero(num_samples_state + num_unstable_modes, num_samples_contr + num_unstable_modes);
	if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		input_map_mat(num_samples_state, num_samples_contr) = 1;
	}
	double time_input = 0.;
	double time_state = 0.;
	int input_sample = 0;
	time_input += inp_sampling_periods_vec[input_sample];
	for (int st_sample = 0; st_sample < num_samples_state; st_sample++) {
		time_state += st_sampling_periods_vec[st_sample];
		if (time_state - kEps > time_input) {
			input_sample++;
			time_input += inp_sampling_periods_vec[input_sample];
		}
		input_map_mat(st_sample, input_sample) = 1.;
	}
	dyn.input_map_mat = input_map_mat;
	dyn.input_map_mat_tr = input_map_mat.transpose();

	// Map input:
	// ----------
	//CommonMatrixType new_input_mat = CommonMatrixType::Zero(ns_st, ns_c + 1);
	// Pos:
	//new_input_mat.block(0, 0, ns_st, ns_c) = dynamics_qp_vec_.front().pos.input_mat.block(0, 0, ns_st, ns_st) * input_map_mat;
	//new_input_mat.block(0, ns_c, ns_st, 1) = dynamics_qp_vec_.front().pos.input_mat.block(0, ns_st, ns_st, 1);
	//dynamics_qp_vec_.front().pos.input_mat = new_input_mat;
	//dynamics_qp_vec_.front().pos.input_mat_tr = new_input_mat.transpose();
	// Vel:
	//new_input_mat.block(0, 0, ns_st, ns_c) = dynamics_qp_vec_.front().vel.input_mat.block(0, 0, ns_st, ns_st) * input_map_mat;
	//new_input_mat.block(0, ns_c, ns_st, 1) = dynamics_qp_vec_.front().vel.input_mat.block(0, ns_st, ns_st, 1);
	//dynamics_qp_vec_.front().vel.input_mat = new_input_mat;
	//dynamics_qp_vec_.front().vel.input_mat_tr = new_input_mat.transpose();
	// Acc:
	//new_input_mat.block(0, 0, ns_st, ns_c) = dynamics_qp_vec_.front().acc.input_mat.block(0, 0, ns_st, ns_st) * input_map_mat;
	//new_input_mat.block(0, ns_c, ns_st, 1) = dynamics_qp_vec_.front().acc.input_mat.block(0, ns_st, ns_st, 1);
	//dynamics_qp_vec_.front().acc.input_mat = new_input_mat;
	//dynamics_qp_vec_.front().acc.input_mat_tr = new_input_mat.transpose();
	// CoP:
	//new_input_mat.block(0, 0, ns_st, ns_c) = dynamics_qp_vec_.front().cop.input_mat.block(0, 0, ns_st, ns_st) * input_map_mat;
	//new_input_mat.block(0, ns_c, ns_st, 1) = dynamics_qp_vec_.front().cop.input_mat.block(0, ns_st, ns_st, 1);
	//dynamics_qp_vec_.front().cop.input_mat = new_input_mat;
	//dynamics_qp_vec_.front().cop.input_mat_tr = new_input_mat.transpose();
	// CP:
	//new_input_mat.block(0, 0, ns_st, ns_c) = dynamics_qp_vec_.front().cp.input_mat.block(0, 0, ns_st, ns_st) * input_map_mat;
	//new_input_mat.block(0, ns_c, ns_st, 1) = dynamics_qp_vec_.front().cp.input_mat.block(0, ns_st, ns_st, 1);
	//dynamics_qp_vec_.front().cp.input_mat = new_input_mat;
	//dynamics_qp_vec_.front().cp.input_mat_tr = new_input_mat.transpose();

}

void DynamicsBuilder::BuildThirdOrder(LinearDynamics &dyn, double height, const std::vector<double> &sampling_periods_vec, int num_samples) {

	dyn.SetZero(3, 1, 1, num_samples, num_samples, 0, 0);

	double sample_period_first = sampling_periods_vec.at(0);
	double sample_period_rest = sampling_periods_vec.at(1);

	BuildThirdOrder(dyn.pos, height, sample_period_first, sample_period_rest, num_samples, POSITION);
	BuildThirdOrder(dyn.vel, height, sample_period_first, sample_period_rest, num_samples, VELOCITY);
	BuildThirdOrder(dyn.acc, height, sample_period_first, sample_period_rest, num_samples, ACCELERATION);
	BuildThirdOrder(dyn.jerk, height, sample_period_first, sample_period_rest, num_samples, JERK);
	BuildThirdOrder(dyn.cop, height, sample_period_first, sample_period_rest, num_samples, COP);

	// Capture Point: \f$ \xi = x + \frac{1}{\omega}\dot x \f$
	dyn.cp.state_mat = dyn.pos.state_mat + 1. / sqrt(kGravity/height) * dyn.vel.state_mat;
	dyn.cp.input_mat = dyn.pos.input_mat + 1. / sqrt(kGravity/height) * dyn.vel.input_mat;
	dyn.cp.input_mat_tr = dyn.cp.input_mat.transpose();
	dyn.cp.input_mat_inv = dyn.cp.input_mat.inverse();
	dyn.cp.input_mat_inv_tr = dyn.cp.input_mat_inv.transpose();
}

void DynamicsBuilder::BuildThirdOrder(LinearDynamicsMatrices &dyn, double height, double sample_period_first, double sample_period_rest, int num_samples, Derivative derivative) {
	assert(height > 0.);
	assert(num_samples > 0.);
	assert(sample_period_first > 0.);
	assert(sample_period_rest > 0.);

	int N = num_samples;
	double s = sample_period_first;
	double T = sample_period_rest;

	switch (derivative){
	case POSITION:
		for (int row = 0; row < N; ++row) {
			dyn.state_mat(row, 0) = 1;
			dyn.state_mat(row, 1) = row * T + s;
			dyn.state_mat(row, 2) = s * s / 2 + row * T * s + row * row * T * T / 2;

			dyn.input_mat(row,0) = dyn.input_mat_tr(0,row) = s*s*s/6 + row*T*s*s/2 + s*(row*row*T*T/2 );
			for (int col=1; col<N; col++) {
				if (col <= row) {
					dyn.input_mat(row,col) = dyn.input_mat_tr(col,row) =T*T*T/6 + 3*(row-col)*T*T*T/6 + 3*(row-col)*(row-col) * T * T * T / 6;
				}
			}
		}
		break;

	case VELOCITY:
		for (int row=0;row<N;row++) {
			dyn.state_mat(row,0) = 0.0;
			dyn.state_mat(row,1) = 1.0;
			dyn.state_mat(row,2) = row*T + s;

			dyn.input_mat(row,0) = dyn.input_mat_tr(0,row) = s*s/2 + row*T*s;
			for (int col=1; col<N; col++) {
				if (col<=row){
					dyn.input_mat(row,col) = dyn.input_mat_tr(col,row) = T*T/2 + (row-col)*T*T;
				}
			}
		}
		break;

	case ACCELERATION:
		for (int row=0; row<N; row++) {
			dyn.state_mat(row,2) = 1.0;

			dyn.input_mat(row,0) = dyn.input_mat_tr(0,row) = s;
			for (int col=1; col<N; col++) {
				if (col<=row){
					dyn.input_mat(row,col) = dyn.input_mat_tr(col,row) = T;
				}
			}
		}
		break;

	case COP:
		for (int row=0; row<N; row++) {
			dyn.state_mat(row,0) = 1;
			dyn.state_mat(row,1) = row*T + s;
			dyn.state_mat(row,2) = s*s/2 + row*T*s + row*row*T*T/2 - height / kGravity;

			dyn.input_mat(row,0) = dyn.input_mat_tr(0,row) =s*s*s/6 + row*T*s*s/2 + s*(row*row*T*T/2 - height / kGravity);
			for(int col=1; col<N; col++){
				if (col <= row) {
					dyn.input_mat(row,col) = dyn.input_mat_tr(col,row) = T*T*T/6 + 3*(row-col)*T*T*T/6 + 3*(row-col)*(row-col)*T*T*T/6 - T * height / kGravity;
				}
			}

		}
		Invert(dyn.input_mat, dyn.input_mat_inv);
		dyn.input_mat_inv_tr=dyn.input_mat_inv.transpose();
		break;

	default:
		dyn.input_mat.setIdentity();
		dyn.input_mat_tr.setIdentity();
		break;
	}
}


void DynamicsBuilder::BuildSecondOrderCoPOutput(LinearDynamicsMatrices &dyn, double height, double sample_period_first, double sample_period_rest, int num_samples, Derivative derivative) {
	assert(height > 0.);
	assert(num_samples > 0.);
	assert(sample_period_first > 0.);
	assert(sample_period_rest > 0.);

	dyn.state_mat.setZero(num_samples, 2);
	dyn.input_mat.setZero(num_samples, num_samples);
	dyn.input_mat_tr.setZero(num_samples, num_samples);
	dyn.input_mat_inv.setZero(num_samples, num_samples);
	dyn.input_mat_inv_tr.setZero(num_samples, num_samples);

	double sp1 = sample_period_first;
	double sp2 = sample_period_rest;
	double sp2sp2 = sp2 * sp2;
	double sp1sp1 = sp1 * sp1;
	switch (derivative){
	case POSITION:
		for (int row = 0; row < num_samples; ++row) {
			dyn.state_mat(row, 0) = 1;
			dyn.state_mat(row, 1) = row * sp2 + sp1;

			dyn.input_mat(row, 0) = dyn.input_mat_tr(0, row) = sp1sp1 / 2 + sp1 * row * sp2;
			for (int col = 1; col <= row; col++) {
				dyn.input_mat(row, col) = dyn.input_mat_tr(col, row) = sp2sp2 / 2 + (row - col) * sp2sp2;
			}
		}
		break;

	case VELOCITY:
		for (int row = 0; row < num_samples; row++) {
			dyn.state_mat(row, 1) = 1.0;

			dyn.input_mat(row, 0) = dyn.input_mat_tr(0, row) = sp1;
			for (int col = 1; col <= row; col++) {
				dyn.input_mat(row, col) = dyn.input_mat_tr(col, row) = sp2;
			}
		}
		break;

	case ACCELERATION:
		dyn.input_mat.setIdentity();
		dyn.input_mat_tr.setIdentity();
		break;

	case COP:
		for (int row = 0; row < num_samples; ++row) {
			dyn.state_mat(row, 0) = 1;
			dyn.state_mat(row, 1) = row * sp2 + sp1;

			dyn.input_mat(row, 0) = dyn.input_mat_tr(0, row) = sp1sp1 / 2 + sp1 * row * sp2;
			for (int col = 1; col <= row; col++) {
				dyn.input_mat(row, col) = dyn.input_mat_tr(col, row) = sp2sp2 / 2 + (row - col) * sp2sp2;
			}

			dyn.input_mat(row, row) = dyn.input_mat_tr(row, row) -= height / kGravity;
		}
		Invert(dyn.input_mat, dyn.input_mat_inv);
		dyn.input_mat_inv_tr = dyn.input_mat_inv.transpose();
		break;

	case JERK:
		break;

	default:
		assert(false);
		break;
	}
}


void DynamicsBuilder::BuildSecondOrderCoPInput(LinearDynamics &dyn, double height, double sample_period_first, double sample_period_rest, int num_samples) {
	assert(height > 0.);
	assert(num_samples > 0.);
	assert(sample_period_first > 0.);
	assert(sample_period_rest > 0.);

	dyn.SetZero(2, 1, 1, num_samples, num_samples, 0, 0);

	double omega 		= sqrt(kGravity/height);
	double omega_square = kGravity/height;

	// Continuous dynamics:
	dyn.cont_ss.c_state_mat(0, 1) 	= 1.;
	dyn.cont_ss.c_state_mat(1, 0) 	= omega_square;
	dyn.cont_ss.c_state_mat_inv 	= dyn.cont_ss.c_state_mat.inverse();
	dyn.cont_ss.c_input_mat(0) 		= 0.;
	dyn.cont_ss.c_input_mat(1) 		= -omega_square;
	// Capture point as output:
	dyn.cont_ss.ss_output_mat(0) 	= 1.;
	dyn.cont_ss.ss_output_mat(1) 	= 1./omega;
	dyn.discr_ss.ss_output_mat 		= dyn.cont_ss.ss_output_mat;
	dyn.discr_ss.ss_output_mat_tr 	= dyn.discr_ss.ss_output_mat.transpose();

	//Eigenvalue decomposition of state matrix: \f[ S e^{\lambda T}S^{-1} \f]
	eigen_solver_.compute(dyn.cont_ss.c_state_mat);
	eigenval_vec_ 		= eigen_solver_.eigenvalues().real();
	eigenvec_mat_ 		= eigen_solver_.eigenvectors().real();
	eigenvec_mat_inv_ 	= eigenvec_mat_.inverse();

	double sp1 = sample_period_first;
	double sp2 = sample_period_rest;
	// position and velocity
	for (int row = 0; row < num_samples; row++) {
		ComputeDiscreteStateMat(dyn, sp1 + row * sp2);
		ComputeDiscreteInputVec(dyn);

		dyn.pos.state_mat.block(row, 0, 1, 2) = dyn.discr_ss.c_state_mat.block(0, 0, 1, 2);
		dyn.vel.state_mat.block(row, 0, 1, 2) = dyn.discr_ss.c_state_mat.block(1, 0, 1, 2);
		for (int col = 0; col + row < num_samples; col++) {
			dyn.pos.input_mat(row + col, col) = dyn.discr_ss.c_input_mat(0);
			dyn.vel.input_mat(row + col, col) = dyn.discr_ss.c_input_mat(1);
		}
	}
	dyn.pos.input_mat_tr = dyn.pos.input_mat.transpose();
	dyn.pos.input_mat_inv = dyn.pos.input_mat.inverse();
	dyn.pos.input_mat_inv_tr = dyn.pos.input_mat_inv.transpose();

	dyn.vel.input_mat_tr = dyn.vel.input_mat.transpose();
	dyn.vel.input_mat_inv = dyn.vel.input_mat.inverse();
	dyn.vel.input_mat_inv_tr = dyn.vel.input_mat_inv.transpose();

	// CoP:
	dyn.cop.input_mat.setIdentity();
	dyn.cop.input_mat_tr.setIdentity();
	dyn.cop.input_mat_inv.setIdentity();
	dyn.cop.input_mat_inv_tr.setIdentity();

	// Acceleration:
	dyn.acc.state_mat = omega_square*(dyn.pos.state_mat - dyn.cop.state_mat);
	dyn.acc.input_mat = omega_square*(dyn.pos.input_mat - dyn.cop.input_mat);

	dyn.acc.input_mat_tr = dyn.acc.input_mat.transpose();
	dyn.acc.input_mat_inv = dyn.acc.input_mat.inverse();
	dyn.acc.input_mat_inv_tr = dyn.acc.input_mat_inv.transpose();

	// CP: \f$ \xi = x + \frac{1}{\omega}\dot x \f$
	dyn.cp.state_mat = dyn.pos.state_mat + 1./omega*dyn.vel.state_mat;
	dyn.cp.input_mat = dyn.pos.input_mat + 1./omega*dyn.vel.input_mat;

	dyn.cp.input_mat_tr = dyn.cp.input_mat.transpose();
	dyn.cp.input_mat_inv = dyn.cp.input_mat.inverse();
	dyn.cp.input_mat_inv_tr = dyn.cp.input_mat_inv.transpose();


	assert(!dyn.pos.input_mat_inv.isZero(kEps) && !dyn.pos.input_mat_tr.isZero(kEps));
	assert(!dyn.vel.input_mat_inv.isZero(kEps) && !dyn.vel.input_mat_tr.isZero(kEps));
	assert(!dyn.cop.input_mat_inv.isZero(kEps) && !dyn.cop.input_mat_tr.isZero(kEps));
	assert(!dyn.acc.input_mat_inv.isZero(kEps) && !dyn.acc.input_mat_tr.isZero(kEps));
	assert(!dyn.cp.input_mat_inv_tr.isZero(kEps) && !dyn.cp.input_mat_tr.isZero(kEps));
}

void DynamicsBuilder::BuildSecondOrderCoPInputGeneral(LinearDynamicsMatrices &dyn_mat, double height, double sample_period_first, double sample_period_rest, int num_samples) {
	assert(height > 0.);
	assert(num_samples > 0.);
	assert(sample_period_first > 0.);
	assert(sample_period_rest > 0.);

	//Eigenvalue decomposition of state matrix: \f[ S e^{\lambda T}S^{-1} \f]
	eigen_solver_.compute(dyn_mat.c_state_mat);
	eigenval_vec_ = eigen_solver_.eigenvalues().real();
	eigenvec_mat_ = eigen_solver_.eigenvectors().real();
	eigenvec_mat_inv_ = eigenvec_mat_.inverse();

	double sp1 = sample_period_first;
	double sp2 = sample_period_rest;

	Matrix2D d_state_mat1 = Matrix2D::Zero();
	CommonMatrixType d_input_mat1;
	ComputeDiscreteStateMatGeneral(d_state_mat1, sp1);
	ComputeDiscreteInputMatGeneral(d_input_mat1, d_state_mat1, dyn_mat);
	Matrix2D d_state_mat2 = Matrix2D::Zero();
	CommonMatrixType d_input_mat2;
	ComputeDiscreteStateMatGeneral(d_state_mat2, sp2);
	ComputeDiscreteInputMatGeneral(d_input_mat2, d_state_mat2, dyn_mat);
	// position and velocity
	dyn_mat.state_mat.block(0, 0, 1, 2) = dyn_mat.ss_output_mat * d_state_mat1;
	dyn_mat.input_mat.block(0, 0, 1, 1) = dyn_mat.ss_output_mat * d_input_mat1;
	Matrix2D d_state_mat_temp = d_state_mat1;
	Matrix2D d_state_mat_temp2 = Matrix2D::Identity();
	//C*Ad2
	CommonMatrixType d_output_mat_temp = dyn_mat.ss_output_mat;
	for (int row = 1; row < num_samples; row++) {
		//C*Ad2^row
		d_output_mat_temp *= d_state_mat2;

		// C * Ad2^row * Ad1
		d_state_mat_temp = d_state_mat2 * d_state_mat_temp;
		// Ad2^row
		d_state_mat_temp2 = d_state_mat2 * d_state_mat_temp2;
		//S(row,:) = C*Ad2^row * Ad1
		dyn_mat.state_mat.block(row, 0, 1, 2) =  d_output_mat_temp * d_state_mat1;
		//U(row,0) = C*Ad2^row * Bd1
		dyn_mat.input_mat.block(row, 0, 1, 1) = d_output_mat_temp * d_input_mat1;
		// U(row, row) = C*Bd2
		dyn_mat.input_mat.block(row, row, 1, 1) = dyn_mat.ss_output_mat * d_input_mat2;
		// Fill diagonal starting at (row, 0)
		for (int col = 1; col + row < num_samples; col++) {
			//U(row+col,col) = C*Ad2^row * Bd2
			dyn_mat.input_mat.block(row + col, col, 1, 1) = d_output_mat_temp * d_input_mat2;
		}
	}

	dyn_mat.input_mat_tr = dyn_mat.input_mat.transpose();
	dyn_mat.input_mat_inv = dyn_mat.input_mat.inverse();
	dyn_mat.input_mat_inv_tr = dyn_mat.input_mat_inv.transpose();

	assert(!dyn_mat.state_mat.isZero(kEps));
	assert(!dyn_mat.input_mat_inv.isZero(kEps) && !dyn_mat.input_mat_tr.isZero(kEps));
}

void DynamicsBuilder::BuildSecondOrderCoPInputDecoupled(LinearDynamicsMatrices &dyn_mat,
		const std::vector<CommonMatrixType> &d_state_mat_vec, const std::vector<CommonMatrixType> &d_input_mat_vec,
		const std::vector<CommonMatrixType> &d_state_mat_pow_vec, const std::vector<CommonMatrixType> &rev_prod_dsmatrices_vec) {
	assert(d_state_mat_vec.size() > 0 && d_input_mat_vec.size() > 0 && d_state_mat_pow_vec.size());

	int num_samples = d_state_mat_vec.size();

	dyn_mat.input_mat(0,0) = dyn_mat.ss_output_mat(0,0) * d_input_mat_vec.at(0)(0, 0);
	//(Su): U(N - 1,N) = Cu
	dyn_mat.input_mat(num_samples - 1, num_samples) = dyn_mat.ss_output_mat(0,1);
	dyn_mat.state_mat(0,0) = dyn_mat.ss_output_mat(0,0) * d_state_mat_vec.at(0)(0,0);

	double Ad1s, Ad2s, Ad2uInv;	//Stable and unstable parts of discretized state matrices
	Ad1s = d_state_mat_vec.at(0)(0,0);
	Ad2s = d_state_mat_vec.at(1)(0,0);	// Only first sampling period is varying
	Ad2uInv = 1. / d_state_mat_vec.at(1)(1,1);
	CommonMatrixType d_state_mat_pow = Matrix2D::Identity();
	double pos_pow_Ad2s = 1.;
	double neg_pow_Ad2u = 1.;
	for (int row = 1; row < num_samples; row++) {
		// Ad2s^row, Ad2u^{-row}
		pos_pow_Ad2s *= Ad2s;
		neg_pow_Ad2u *= Ad2uInv;
		d_state_mat_pow(0, 0) = pos_pow_Ad2s;
		d_state_mat_pow(1, 1) = neg_pow_Ad2u;

		// Build augmented state matrix:
		// -----------------------------
		//Ss(1:N, 0) = Cs * Ad2s^row;
		dyn_mat.state_mat(row, 0) = dyn_mat.ss_output_mat(0, 0) * d_state_mat_pow(0, 0) * Ad1s;

		// Build augmented input matrix:
		// -----------------------------
		//U(row, row) = Cs * B1s
		dyn_mat.input_mat(row, row) = dyn_mat.ss_output_mat(0, 0) * d_input_mat_vec.at(0)(0, 0);
		//(Su): U(N - 1 - row, N) = Cu*Ad2u^(-row);
		dyn_mat.input_mat(num_samples - 1 - row, num_samples) = dyn_mat.ss_output_mat(0, 1) * d_state_mat_pow(1, 1);//new * d_state_mat_vec.at(1)(1, 1);
		//U(row, 0) = Cs * Ad2s^row * Bd1s
		dyn_mat.input_mat(row, 0) = dyn_mat.ss_output_mat(0, 0) * d_state_mat_pow(0,0) * d_input_mat_vec.at(0)(0);
		//U(0, row) = -Cu * Ad2u^row * Bd2u
		dyn_mat.input_mat(0, row) = - dyn_mat.ss_output_mat(0, 1) * d_state_mat_pow(1,1) * d_input_mat_vec.at(1)(1);
		// Fill diagonal starting at (row + 1, 1)
		for (int col = 1; col + row < num_samples; col++) {
			//U(row+col, col) = Cs * Ad2s^row * Bd2s
			dyn_mat.input_mat(row + col, col) = dyn_mat.ss_output_mat(0,0) * d_state_mat_pow(0,0) * d_input_mat_vec.at(1)(0);
			//U(col, col+row) = Cu * Ad2u^row * Bd2u
			dyn_mat.input_mat(col, col + row) = -dyn_mat.ss_output_mat(0,1) * d_state_mat_pow(1,1) * d_input_mat_vec.at(1)(1);
		}
	}

	// Build augmented (stable) state matrix:
	// --------------------------------------
	for (int row = 0; row < num_samples; row++) {
		dyn_mat.state_mat(row, 0) = dyn_mat.ss_output_mat(0, 0) * d_state_mat_pow_vec.at(row)(0, 0);
	}

	// Build stable dynamics in the lower left corner of the augmented input matrix:
	// -----------------------------------------------------------------------------
	double mat_product = 0.;
	for (int col = 0; col < num_samples; col++) {
		mat_product = dyn_mat.ss_output_mat(0, 0) * d_input_mat_vec.at(col)(0, 0);
		//U(col, col) = Cs * Bs_{col}
		dyn_mat.input_mat(col, col) = mat_product;
		for (int row = col + 1; row < num_samples; row++) {
			mat_product *= d_state_mat_vec.at(row)(0, 0);
			//U(row, col) = Cs * As_{row} * Bs_{col}
			dyn_mat.input_mat(row, col) = mat_product;
		}
	}

	// Build augmented unstable state matrix and put in the last colum of the augmented input matrix:
	// ----------------------------------------------------------------------------------------------
	for (int row = 0; row < num_samples - 1; row++) {
		dyn_mat.input_mat(row, num_samples) = dyn_mat.ss_output_mat(0, 1) / rev_prod_dsmatrices_vec.at(num_samples - 2 - row)(1, 1);
	}
	dyn_mat.input_mat(num_samples - 1, num_samples) = dyn_mat.ss_output_mat(0, 1);

	// Build unstable dynamics in the upper right corner of the augmented input matrix:
	// -------------------------------------------------------------------------------
	mat_product = 0.;
	for (int col = num_samples - 1; col > 0; col--) {
		mat_product = - dyn_mat.ss_output_mat(0, 1) / d_state_mat_vec.at(col)(1, 1);
		dyn_mat.input_mat(col - 1, col) = mat_product * d_input_mat_vec.at(col)(1, 0);
		for (int row = col - 2; row >= 0; row--) {
			mat_product /= d_state_mat_vec.at(row + 1)(1, 1);
			dyn_mat.input_mat(row, col) = mat_product * d_input_mat_vec.at(col)(1, 0);
		}
	}


	dyn_mat.input_mat_tr.noalias() = dyn_mat.input_mat.transpose();
	//dyn_mat.input_mat_inv = dyn_mat.input_mat.inverse();
	//dyn_mat.input_mat_inv_tr = dyn_mat.input_mat_inv.transpose();


	assert(!dyn_mat.input_mat.isZero(kEps));
}

void DynamicsBuilder::ComputeDiscreteSSDynamics(LinearDynamics &dyn, const std::vector<double> &sampling_periods_vec) {
	assert(sampling_periods_vec.size() > 0);

	//Eigenvalue decomposition of a diagonal state matrix: \f[ S e^{\lambda T}S^{-1} \f]
	eigen_solver_.compute(dyn.cont_ss.c_state_mat);
	eigenval_vec_ = eigen_solver_.eigenvalues().real();
	eigenvec_mat_ = eigen_solver_.eigenvectors().real();
	eigenvec_mat_inv_ = eigenvec_mat_.inverse();

	ComputeDiscreteStateMatDecoupled(dyn.d_input_mat_vec[0], dyn.d_state_mat_vec[0], sampling_periods_vec[0], dyn.cont_ss);
	// Ad_pow(0) = A_0
	dyn.d_state_mat_pow_vec[0] = dyn.d_state_mat_vec[0];
	// Ad_pow(0) = I
	dyn.d_state_mat_pow2_vec[0] = CommonMatrixType::Identity(2, 2);
	//ComputeDiscreteInputMatGeneral(dyn.d_input_mat_vec[0], dyn.d_state_mat_vec[0], dyn.cont_ss);
	for (unsigned period_num = 1; period_num < sampling_periods_vec.size(); period_num++) {
		ComputeDiscreteStateMatDecoupled(dyn.d_input_mat_vec[period_num], dyn.d_state_mat_vec[period_num], sampling_periods_vec[period_num], dyn.cont_ss);
		// A_{i-1}*A_i
		dyn.d_state_mat_pow_vec[period_num](0, 0) = dyn.d_state_mat_pow_vec[period_num - 1](0, 0) * dyn.d_state_mat_vec[period_num](0, 0);
		dyn.d_state_mat_pow_vec[period_num](1, 1) = dyn.d_state_mat_pow_vec[period_num - 1](1, 1) * dyn.d_state_mat_vec[period_num](1, 1);
		dyn.d_state_mat_pow2_vec[period_num](0, 0) = dyn.d_state_mat_pow2_vec[period_num - 1](0, 0) * dyn.d_state_mat_vec[period_num](0, 0);
		dyn.d_state_mat_pow2_vec[period_num](1, 1) = dyn.d_state_mat_pow2_vec[period_num - 1](1, 1) * dyn.d_state_mat_vec[period_num](1, 1);
		//ComputeDiscreteInputMatGeneral(dyn.d_input_mat_vec[period_num], dyn.d_state_mat_vec[period_num], dyn.cont_ss);
	}
}

void DynamicsBuilder::BuildStateMatrixProducts(std::vector<CommonMatrixType> &d_state_mat_pow_vec, std::vector<CommonMatrixType> &rev_prod_dsmatrices_vec,
		const std::vector<CommonMatrixType> &d_state_mat_vec) {
	assert(d_state_mat_vec.size() > 0);

	const int num_matrices = d_state_mat_vec.size();

	// Forward:
	// --------
	d_state_mat_pow_vec.at(0) = d_state_mat_vec.at(0);
	for (int mat_num = 1; mat_num < num_matrices; mat_num++) {
		// A_{i-1}*A_i
		d_state_mat_pow_vec.at(mat_num) = d_state_mat_pow_vec.at(mat_num - 1) * d_state_mat_vec.at(mat_num);
	}

	// Reverse order:
	// --------------
	rev_prod_dsmatrices_vec.at(0)(0, 0) = d_state_mat_vec.at(num_matrices - 1)(0, 0); // Start from last matrix
	rev_prod_dsmatrices_vec.at(0)(1, 1) = d_state_mat_vec.at(num_matrices - 1)(1, 1);
	for (int mat_num = 1; mat_num < num_matrices; mat_num++) {
		rev_prod_dsmatrices_vec.at(mat_num)(0, 0) = rev_prod_dsmatrices_vec.at(mat_num - 1)(0, 0) * d_state_mat_vec.at(num_matrices - 1 - mat_num)(0, 0);
		rev_prod_dsmatrices_vec.at(mat_num)(1, 1) = rev_prod_dsmatrices_vec.at(mat_num - 1)(1, 1) * d_state_mat_vec.at(num_matrices - 1 - mat_num)(1, 1);
	}
}

void DynamicsBuilder::ComputeDiscreteStateMatGeneral(Matrix2D &d_state_mat, double sample_period) {
	diag_exp_eig_mat_(0, 0) = exp(eigenval_vec_[0] * sample_period);
	diag_exp_eig_mat_(1, 1) = exp(eigenval_vec_[1] * sample_period);

	d_state_mat.noalias() = eigenvec_mat_ * diag_exp_eig_mat_ * eigenvec_mat_inv_;

	assert(!d_state_mat.isZero(kEps));
}

void DynamicsBuilder::ComputeDiscreteInputMatGeneral(CommonMatrixType &d_input_mat, const CommonMatrixType &d_state_mat, const LinearDynamicsMatrices &dyn_mat) {
	tmp_vec_.noalias() = -dyn_mat.c_state_mat_inv * identity_mat_ * dyn_mat.c_input_mat ;
	d_input_mat.noalias() = dyn_mat.c_state_mat_inv * d_state_mat * dyn_mat.c_input_mat + tmp_vec_;

	assert(!d_input_mat.isZero(kEps));
}

void DynamicsBuilder::ComputeDiscreteStateMatDecoupled(CommonMatrixType &d_input_mat, CommonMatrixType &d_state_mat, double sample_period, const LinearDynamicsMatrices &dyn_mat) {
	assert(sample_period > 0.);
	assert(!eigenval_vec_.isZero(kEps));

	d_state_mat(0, 0) = exp(eigenval_vec_[0] * sample_period);
	d_state_mat(1, 1) = exp(eigenval_vec_[1] * sample_period);

	Matrix2D expm1_state_mat = Matrix2D::Zero();
	expm1_state_mat(0, 0) = exp(eigenval_vec_[0] * sample_period) - 1.;
	expm1_state_mat(1, 1) = exp(eigenval_vec_[1] * sample_period) - 1.;

	d_input_mat.noalias() = dyn_mat.c_state_mat_inv * expm1_state_mat * dyn_mat.c_input_mat;

	assert(!d_input_mat.isZero(kEps));
	assert(!d_state_mat.isZero(kEps));
}

void DynamicsBuilder::ComputeDiscreteStateMat(LinearDynamics &dyn, double sample_period) {
	assert(sample_period > 0.);
	assert(!eigenval_vec_.isZero());

	diag_exp_eig_mat_(0, 0) = exp(eigenval_vec_[0] * sample_period);
	diag_exp_eig_mat_(1, 1) = exp(eigenval_vec_[1] * sample_period);

	dyn.discr_ss.c_state_mat.noalias() = eigenvec_mat_ * diag_exp_eig_mat_ * eigenvec_mat_inv_;
}

void DynamicsBuilder::ComputeDiscreteInputVec(LinearDynamics &dyn) {
	tmp_vec_.noalias() = -dyn.cont_ss.c_state_mat_inv  * identity_mat_ * dyn.cont_ss.c_input_mat;
	dyn.discr_ss.c_input_mat.noalias() = dyn.cont_ss.c_state_mat_inv * dyn.discr_ss.c_state_mat * dyn.cont_ss.c_input_mat + tmp_vec_;
	dyn.discr_ss.c_input_mat_tr = dyn.discr_ss.c_input_mat.transpose();
}


// Tests:
// ------
void DynamicsBuilder::RecoverStandardInputMat(CommonMatrixType &h_mat, const CommonMatrixType &input_mat,
		const std::vector<CommonMatrixType> &rev_prod_dsmatrices_vec, const std::vector<CommonMatrixType> &d_input_mat_vec) {

	int num_samples = d_input_mat_vec.size();

	// Build lu_vec:
	CommonVectorType lu_vec = CommonVectorType::Zero(num_samples);
	for (int i = 0; i < num_samples - 1; i++) {
		lu_vec(i) = rev_prod_dsmatrices_vec.at(num_samples - 2 - i)(1, 1) * d_input_mat_vec.at(i)(1);
	}
	lu_vec(num_samples - 1) = d_input_mat_vec.at(num_samples - 1)(1);

	// Build lambdau_lu_mat:
	CommonMatrixType lambda_lu_mat = CommonMatrixType::Zero(num_samples, num_samples);
	double lu_val = 0;
	for (int col = 0; col < num_samples; col++) {
		lu_val = lu_vec(col);
		for (int row = 0; row < num_samples; row++) {
			lambda_lu_mat(row, col) = input_mat(row, num_samples) * lu_val;
		}
	}

	CommonMatrixType lambda_lu_mat_tr = lambda_lu_mat.transpose();
	CommonMatrixType input_mat_tr = input_mat.transpose();

	// Build normalized hessian:
	// -------------------------
	CommonMatrixType hn_mat = input_mat_tr.block(0, 0, num_samples, num_samples) * input_mat.block(0, 0, num_samples, num_samples);

	// Build standard hessian:
	// -----------------------
	h_mat = hn_mat
			+ input_mat_tr.block(0, 0, num_samples, num_samples) * lambda_lu_mat
			+ lambda_lu_mat_tr * input_mat.block(0, 0, num_samples, num_samples)
			+ lambda_lu_mat_tr * lambda_lu_mat;

}
