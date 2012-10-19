#include <mpc-walkgen/dynamics-builder.h>
#include <mpc-walkgen/tools.h>

#include <math.h>
#include <iostream>

using namespace MPCWalkgen;

//
// Public methods:
//
DynamicsBuilder::DynamicsBuilder() {
	Init();
}

DynamicsBuilder::~DynamicsBuilder() {}

void DynamicsBuilder::Init() {
	eigenval_vec_.setZero(); eigenvec_mat_.setZero(); eigenvec_mat_inv_.setZero();
	diag_exp_eig_mat_.setZero();

	identity_mat_.setIdentity();

	precomp_input_mat_.setZero();  // \f[ -A^{-1}\mathbb{I}B \f]

}

void DynamicsBuilder::Build(DynamicsOrder dynamics_order, LinearDynamics &dyn, double height, double sample_period_first, double sample_period_rest, int num_samples) {
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

	BuildSecondOrderCoPInput(dyn, height, sample_period_first, sample_period_rest, num_samples);
}

void DynamicsBuilder::BuildThirdOrder(LinearDynamics &dyn, double height, double sample_period_first, double sample_period_rest, int num_samples) {
	dyn.SetZero(3, 1, 1, num_samples);

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


	dyn.SetZero(2, 1, 1, num_samples);

	double omega = sqrt(kGravity/height);
	double omega_square = kGravity/height;

	// Continuous dynamics:
	dyn.cont_ss.ss_state_mat(0, 1) = 1.; dyn.cont_ss.ss_state_mat(1, 0) = omega_square;
	dyn.cont_ss.ss_state_mat_inv = dyn.cont_ss.ss_state_mat.inverse();
	dyn.cont_ss.ss_input_mat(0) = 0.; dyn.cont_ss.ss_input_mat(1) = - omega_square;

	//Eigenvalue decomposition of cont_state_mat_: \f[ S e^{\lambda T}S^{-1} \f]
	eigen_solver_.compute(dyn.cont_ss.ss_state_mat);
	eigenval_vec_ = eigen_solver_.eigenvalues().real();
	eigenvec_mat_ = eigen_solver_.eigenvectors().real();
	eigenvec_mat_inv_ = eigenvec_mat_.inverse();

	double sp1 = sample_period_first;
	double sp2 = sample_period_rest;
	double sp2sp2 = sp2 * sp2;
	double sp1sp1 = sp1 * sp1;
	//case POSITION, VELOCITY:
	for (int row = 0; row < num_samples; row++) {
		ComputeDiscreteStateMat(dyn, sp1 + row * sp2);
		ComputeDiscreteInputVec(dyn);

		dyn.pos.state_mat.block(row, 0, 1, 2) = dyn.discr_ss.ss_state_mat.block(0, 0, 1, 2);
		dyn.vel.state_mat.block(row, 0, 1, 2) = dyn.discr_ss.ss_state_mat.block(1, 0, 1, 2);
		for (int col = 0; col + row < num_samples; col++) {
			dyn.pos.input_mat(row + col, col) = dyn.discr_ss.ss_input_mat(0);
			dyn.vel.input_mat(row + col, col) = dyn.discr_ss.ss_input_mat(1);
		}
	}
	dyn.pos.input_mat_tr = dyn.pos.input_mat.transpose();
	dyn.pos.input_mat_inv = dyn.pos.input_mat.inverse();
	dyn.pos.input_mat_inv_tr = dyn.pos.input_mat_inv.transpose();

	dyn.vel.input_mat_tr = dyn.vel.input_mat.transpose();
	dyn.vel.input_mat_inv = dyn.vel.input_mat.inverse();
	dyn.vel.input_mat_inv_tr = dyn.vel.input_mat_inv.transpose();

	// COP:
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

	// Capture Point: \f$ \xi = x + \frac{1}{\omega}\dot x \f$
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


void DynamicsBuilder::ComputeDiscreteStateMat(LinearDynamics &dyn, double sample_period) {
	diag_exp_eig_mat_(0, 0) = exp(eigenval_vec_[0] * sample_period);
	diag_exp_eig_mat_(1, 1) = exp(eigenval_vec_[1] * sample_period);

	dyn.discr_ss.ss_state_mat.noalias() = eigenvec_mat_ * diag_exp_eig_mat_ * eigenvec_mat_inv_;
}

void DynamicsBuilder::ComputeDiscreteInputVec(LinearDynamics &dyn) {
	tmp_vec_.noalias() = -dyn.cont_ss.ss_state_mat_inv  * identity_mat_ * dyn.cont_ss.ss_input_mat ;
	dyn.discr_ss.ss_input_mat.noalias() = dyn.cont_ss.ss_state_mat_inv * dyn.discr_ss.ss_state_mat * dyn.cont_ss.ss_input_mat + tmp_vec_;
}

