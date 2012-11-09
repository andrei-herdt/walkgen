#include <mpc-walkgen/dynamics-builder.h>
#include <mpc-walkgen/tools.h>
#include <mpc-walkgen/debug.h>

#include <math.h>

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

void DynamicsBuilder::Build(SystemOrder dynamics_order, LinearDynamics &dyn, double height, double sample_period_first, double sample_period_rest, int num_samples) {
	switch (dynamics_order) {
	case SECOND_ORDER:
		BuildSecondOrder(dyn, height, sample_period_first, sample_period_rest, num_samples);
		break;
	case THIRD_ORDER:
		BuildThirdOrder(dyn, height, sample_period_first, sample_period_rest, num_samples);
		break;
	}
}

//
// Private methods:
//
void DynamicsBuilder::BuildSecondOrder(LinearDynamics &dyn, double height, double sample_period_first, double sample_period_rest, int num_samples) {
	/*
	BuildSecondOrderCoPOutput(dyn.pos, height, sample_period_first, sample_period_rest, num_samples, POSITION);
	BuildSecondOrderCoPOutput(dyn.vel, height, sample_period_first, sample_period_rest, num_samples, VELOCITY);
	BuildSecondOrderCoPOutput(dyn.acc, height, sample_period_first, sample_period_rest, num_samples, ACCELERATION);
	BuildSecondOrderCoPOutput(dyn.cop, height, sample_period_first, sample_period_rest, num_samples, COP);
	 */

	double omega = sqrt(kGravity/height);
	double omega_square = kGravity/height;

	if (mpc_parameters_->formulation == STANDARD) {
		dyn.SetZero(2, 1, 1, num_samples, 0, 0);
		dyn.cont_ss.c_state_mat(0, 1) = 1.;
		dyn.cont_ss.c_state_mat(1, 0) = omega_square;
		dyn.cont_ss.c_state_mat_inv = dyn.cont_ss.c_state_mat.inverse();
		dyn.cont_ss.c_input_mat(0) = 0.;
		dyn.cont_ss.c_input_mat(1) = - omega_square;

		dyn.pos.ss_output_mat(0, 0) = 1.;
		dyn.vel.ss_output_mat(0, 1) = 1.;
		dyn.cp.ss_output_mat(0, 0) = 1.;
		dyn.cp.ss_output_mat(0, 1) = 1./omega;
	} else if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		int num_stable_modes = 1;
		int num_unstable_modes = 1;
		dyn.SetZero(2, 1, 1, num_samples, num_stable_modes, num_unstable_modes);
		// Continuous (general) state space dynamics:
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
	}

	dyn.pos.c_state_mat 	= dyn.cont_ss.c_state_mat;
	dyn.pos.c_state_mat_inv = dyn.cont_ss.c_state_mat_inv;
	dyn.pos.c_input_mat 	= dyn.cont_ss.c_input_mat;
	//BuildSecondOrderCoPInputDecoupled(dyn.pos, height, sample_period_first, sample_period_rest, num_samples);
	//Debug::Cout("dyn.pos.input_mat_dec",dyn.pos.input_mat);
	//Debug::WriteToFile("Uposd", sample_period_first, "dat", dyn.pos.input_mat);
	//dyn.pos.input_mat.setZero();
	BuildSecondOrderCoPInputGeneral(dyn.pos, height, sample_period_first, sample_period_rest, num_samples);
	Debug::Cout("dyn.pos.input_mat",dyn.pos.input_mat);
	Debug::WriteToFile("Upos", sample_period_first, "dat", dyn.pos.input_mat);

	dyn.vel.c_state_mat 	= dyn.cont_ss.c_state_mat;
	dyn.vel.c_state_mat_inv = dyn.cont_ss.c_state_mat_inv;
	dyn.vel.c_input_mat 	= dyn.cont_ss.c_input_mat;
	BuildSecondOrderCoPInputGeneral(dyn.vel, height, sample_period_first, sample_period_rest, num_samples);

	dyn.cp.c_state_mat 		= dyn.cont_ss.c_state_mat;
	dyn.cp.c_state_mat_inv 	= dyn.cont_ss.c_state_mat_inv;
	dyn.cp.c_input_mat 		= dyn.cont_ss.c_input_mat;
	BuildSecondOrderCoPInputGeneral(dyn.cp, height, sample_period_first, sample_period_rest, num_samples);

	dyn.cop.input_mat.setIdentity();
	dyn.cop.input_mat_tr.setIdentity();
	dyn.cop.input_mat_inv.setIdentity();
	dyn.cop.input_mat_inv_tr.setIdentity();

	dyn.acc.state_mat = omega_square*(dyn.pos.state_mat - dyn.cop.state_mat);
	dyn.acc.input_mat = omega_square*(dyn.pos.input_mat - dyn.cop.input_mat);
}

void DynamicsBuilder::BuildThirdOrder(LinearDynamics &dyn, double height, double sample_period_first, double sample_period_rest, int num_samples) {
	dyn.SetZero(3, 1, 1, num_samples, 0, 0);

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

	dyn.SetZero(2, 1, 1, num_samples, 0, 0);

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
	ComputeDiscreteInputVecGeneral(d_input_mat1, d_state_mat1, dyn_mat);
	Matrix2D d_state_mat2 = Matrix2D::Zero();
	CommonMatrixType d_input_mat2;
	ComputeDiscreteStateMatGeneral(d_state_mat2, sp2);
	ComputeDiscreteInputVecGeneral(d_input_mat2, d_state_mat2, dyn_mat);
	// position and velocity
	dyn_mat.input_mat.block(0, 0, 1, 1) = dyn_mat.ss_output_mat * d_input_mat1;
	dyn_mat.state_mat.block(0, 0, 1, 2) = dyn_mat.ss_output_mat * d_state_mat1;
	Matrix2D d_state_mat_temp = Matrix2D::Identity();
	for (int row = 1; row < num_samples; row++) {
		// Ad2^row
		d_state_mat_temp *= d_state_mat2;
		//S(row,:) = C*Ad2^row * Ad1
		dyn_mat.state_mat.block(row, 0, 1, 2) = dyn_mat.ss_output_mat * d_state_mat_temp * d_state_mat1;
		//U(row,0) = C*Ad2^row * Bd1
		dyn_mat.input_mat.block(row, 0, 1, 1) = dyn_mat.ss_output_mat * d_state_mat_temp * d_input_mat1;
		// U(row, row) = C*Bd2
		dyn_mat.input_mat.block(row, row, 1, 1) = dyn_mat.ss_output_mat * d_input_mat2;
		// Fill diagonal starting at (row, 0)
		for (int col = 1; col + row < num_samples; col++) {
			//U(row+col,col) = C*Ad2^row * Bd2
			dyn_mat.input_mat.block(row + col, col, 1, 1) = dyn_mat.ss_output_mat * d_state_mat_temp * d_input_mat2;
		}
	}

	dyn_mat.input_mat_tr = dyn_mat.input_mat.transpose();
	dyn_mat.input_mat_inv = dyn_mat.input_mat.inverse();
	dyn_mat.input_mat_inv_tr = dyn_mat.input_mat_inv.transpose();

	assert(!dyn_mat.state_mat.isZero(kEps));
	assert(!dyn_mat.input_mat_inv.isZero(kEps) && !dyn_mat.input_mat_tr.isZero(kEps));
}

void DynamicsBuilder::BuildSecondOrderCoPInputDecoupled(LinearDynamicsMatrices &dyn_mat, double height, double sample_period_first, double sample_period_rest, int num_samples) {
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
	ComputeDiscreteStateMatDecoupled(d_state_mat1, sp1);
	ComputeDiscreteInputVecGeneral(d_input_mat1, d_state_mat1, dyn_mat);
	Matrix2D d_state_mat2 = Matrix2D::Zero();
	CommonMatrixType d_input_mat2;
	ComputeDiscreteStateMatDecoupled(d_state_mat2, sp2);
	ComputeDiscreteInputVecGeneral(d_input_mat2, d_state_mat2, dyn_mat);
	// position and velocity
	dyn_mat.input_mat(0,0) = dyn_mat.ss_output_mat(0,0) * d_input_mat1(0, 0);
	dyn_mat.stab_state_mat(0,0) = dyn_mat.ss_output_mat(0,0) * d_state_mat1(0,0);
	dyn_mat.unst_state_mat(num_samples - 1, 0) = dyn_mat.ss_output_mat(0,1) * d_state_mat1(1,1);
	Matrix2D d_state_mat_temp = Matrix2D::Identity();
	for (int row = 1; row < num_samples; row++) {
		// Ad2^row
		d_state_mat_temp *= d_state_mat2;
		//S(row,:) = C*Ad2^row * Ad1
		dyn_mat.stab_state_mat(0, 0) = dyn_mat.ss_output_mat(0, 0) * d_state_mat_temp(0, 0) * d_state_mat1(0, 0);
		dyn_mat.unst_state_mat(num_samples - 1, 0) = dyn_mat.ss_output_mat(0,1) * d_state_mat_temp(1,1) * d_state_mat1(1,1);
		//U(row,0) = C*Ad2^row * Bd1
		dyn_mat.input_mat(row, 0) = dyn_mat.ss_output_mat(0,0) * d_state_mat_temp(0,0) * d_input_mat1(0);
		dyn_mat.input_mat(0, row) = dyn_mat.ss_output_mat(0,1) * d_state_mat_temp(1,1) * d_input_mat2(1);
		// Fill diagonal starting at (row, 0)
		for (int col = 1; col + row < num_samples; col++) {
			//U(row+col,col) = Cs * Ad2s^row * Bd2s
			dyn_mat.input_mat(row + col, col) = dyn_mat.ss_output_mat(0,0) * d_state_mat_temp(0,0) * d_input_mat2(0);
			//U(col,col+row) = Cu * Ad2u^row * Bd2u
			dyn_mat.input_mat(col, col + row) = dyn_mat.ss_output_mat(0,1) * d_state_mat_temp(1,1) * d_input_mat2(1);
		}
	}

	dyn_mat.input_mat_tr = dyn_mat.input_mat.transpose();
	dyn_mat.input_mat_inv = dyn_mat.input_mat.inverse();
	dyn_mat.input_mat_inv_tr = dyn_mat.input_mat_inv.transpose();

	assert(!dyn_mat.input_mat_inv.isZero(kEps) && !dyn_mat.input_mat_tr.isZero(kEps));
}

void DynamicsBuilder::ComputeDiscreteStateMatGeneral(Matrix2D &d_state_mat, double sample_period) {
	diag_exp_eig_mat_(0, 0) = exp(eigenval_vec_[0] * sample_period);
	diag_exp_eig_mat_(1, 1) = exp(eigenval_vec_[1] * sample_period);

	d_state_mat.noalias() = eigenvec_mat_ * diag_exp_eig_mat_ * eigenvec_mat_inv_;
}

void DynamicsBuilder::ComputeDiscreteInputVecGeneral(CommonMatrixType &d_input_mat, const Matrix2D &d_state_mat, const LinearDynamicsMatrices &dyn_mat) {
	tmp_vec_.noalias() = -dyn_mat.c_state_mat_inv * identity_mat_ * dyn_mat.c_input_mat ;
	d_input_mat.noalias() = dyn_mat.c_state_mat_inv * d_state_mat * dyn_mat.c_input_mat + tmp_vec_;
}

void DynamicsBuilder::ComputeDiscreteStateMatDecoupled(Matrix2D &d_state_mat, double sample_period) {
	d_state_mat(0, 0) = exp(eigenval_vec_[0] * sample_period);
	d_state_mat(1, 1) = exp(-eigenval_vec_[1] * sample_period);
}

void DynamicsBuilder::ComputeDiscreteStateMat(LinearDynamics &dyn, double sample_period) {
	diag_exp_eig_mat_(0, 0) = exp(eigenval_vec_[0] * sample_period);
	diag_exp_eig_mat_(1, 1) = exp(eigenval_vec_[1] * sample_period);

	dyn.discr_ss.c_state_mat.noalias() = eigenvec_mat_ * diag_exp_eig_mat_ * eigenvec_mat_inv_;
}

void DynamicsBuilder::ComputeDiscreteInputVec(LinearDynamics &dyn) {
	tmp_vec_.noalias() = -dyn.cont_ss.c_state_mat_inv  * identity_mat_ * dyn.cont_ss.c_input_mat ;
	dyn.discr_ss.c_input_mat.noalias() = dyn.cont_ss.c_state_mat_inv * dyn.discr_ss.c_state_mat * dyn.cont_ss.c_input_mat + tmp_vec_;
	dyn.discr_ss.c_input_mat_tr = dyn.discr_ss.c_input_mat.transpose();
}

