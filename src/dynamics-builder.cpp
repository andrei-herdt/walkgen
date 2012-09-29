#include <mpc-walkgen/dynamics-builder.h>
#include <mpc-walkgen/tools.h>

#include <math.h>

using namespace MPCWalkgen;

//
// Public methods:
//
DynamicsBuilder::DynamicsBuilder() {}

DynamicsBuilder::~DynamicsBuilder() {}

void DynamicsBuilder::Init() {
	identity_mat_.setIdentity();
	cont_state_mat_.setZero(); cont_input_vec_.setZero();

	diag_exp_eig_mat_.setZero();
}

void DynamicsBuilder::Build(DynamicsOrder dynamics_order, LinearDynamics &dyn, double height, double sample_period_first, 
		double sample_period_rest, int num_samples)
{

	switch (dynamics_order){
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
	BuildSecondOrder(dyn.pos, height, sample_period_first, sample_period_rest, num_samples, POSITION);
	BuildSecondOrder(dyn.vel, height, sample_period_first, sample_period_rest, num_samples, VELOCITY);
	BuildSecondOrder(dyn.acc, height, sample_period_first, sample_period_rest, num_samples, ACCELERATION);
	BuildSecondOrder(dyn.cop, height, sample_period_first, sample_period_rest, num_samples, COP);
}

void DynamicsBuilder::BuildThirdOrder(LinearDynamics &dyn, double height, double sample_period_first, double sample_period_rest, int num_samples) {
	BuildThirdOrder(dyn.pos, height, sample_period_first, sample_period_rest, num_samples, POSITION);
	BuildThirdOrder(dyn.vel, height, sample_period_first, sample_period_rest, num_samples, VELOCITY);
	BuildThirdOrder(dyn.acc, height, sample_period_first, sample_period_rest, num_samples, ACCELERATION);
	BuildThirdOrder(dyn.jerk, height, sample_period_first, sample_period_rest, num_samples, JERK);
	BuildThirdOrder(dyn.cop, height, sample_period_first, sample_period_rest, num_samples, COP);
}

void DynamicsBuilder::BuildThirdOrder(LinearDynamicsMatrices &dyn, double height,
		double sample_period_first, double sample_period_rest, int num_samples, Derivative derivative)
{
	assert(height > 0.);
	assert(num_samples > 0.);
	assert(sample_period_first > 0.);
	assert(sample_period_rest > 0.);

	const double kGravity = 9.81;

	int N = num_samples;
	double s = sample_period_first;
	double T = sample_period_rest;

	dyn.state_mat.setZero(N, 3);
	dyn.input_mat.setZero(N, N);
	dyn.input_mat_tr.setZero(N, N);
	dyn.input_mat_inv.setZero(N, N);
	dyn.input_mat_inv_tr.setZero(N, N);

	switch (derivative){
	case POSITION:
		for (int row = 0; row < N; ++row) {
			dyn.state_mat(row,0) = 1;
			dyn.state_mat(row,1) = row * T + s;
			dyn.state_mat(row,2) = s * s / 2 + row * T * s + row * row * T * T / 2;

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
		inverse(dyn.input_mat, dyn.input_mat_inv);
		dyn.input_mat_inv_tr=dyn.input_mat_inv.transpose();
		break;

	default:
		dyn.input_mat.setIdentity();
		dyn.input_mat_tr.setIdentity();
		break;
	}
}


void DynamicsBuilder::BuildSecondOrder(LinearDynamicsMatrices &dyn, double height,
		double sample_period_first, double sample_period_rest,
		int num_samples, Derivative derivative)
{
	assert(height > 0.);
	assert(num_samples > 0.);
	assert(sample_period_first > 0.);
	assert(sample_period_rest > 0.);

	const double kGravity = 9.81;

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
		inverse(dyn.input_mat, dyn.input_mat_inv);
		dyn.input_mat_inv_tr = dyn.input_mat_inv.transpose();
		break;

	case JERK:
		break;

	default:
		assert(false);
		break;
	}
}


void DynamicsBuilder::BuildSecondOrderCoP(LinearDynamics &dyn, double height,
		double sample_period_first, double sample_period_rest,
		int num_samples, Derivative derivative) {
	assert(height > 0.);
	assert(num_samples > 0.);
	assert(sample_period_first > 0.);
	assert(sample_period_rest > 0.);

	dyn.SetZero(2, num_samples);

	cont_state_mat_(0,1) = 1.; cont_state_mat_(1,0) = kGravity / height;
	cont_state_mat_inv_ = cont_state_mat_.inverse();

	cont_input_vec_(1) = -kGravity / height;

	//Diagonal matrix: \f[ S e^{\lambda T}S^{-1} \f]
	Eigen::SelfAdjointEigenSolver<Matrix2D> eigensolver(cont_state_mat_);
	assert(eigensolver.info() == Eigen::Success);
	eigenval_vec_ = eigensolver.eigenvalues();
	eigenvec_mat_ = eigensolver.eigenvectors();
	eigenvec_mat_inv_ = eigenvec_mat_.inverse();

	Matrix2D discr_state_mat;
	Matrix2D discr_input_mat, discr_input_mat_tr;

	double sp1 = sample_period_first;
	double sp2 = sample_period_rest;
	double sp2sp2 = sp2 * sp2;
	double sp1sp1 = sp1 * sp1;
	//case POSITION, VELOCITY:
	for (int row = 0; row < num_samples; row++) {
		ComputeDiscreteStateMat(discr_state_mat, sp1 + row*sp2);
		ComputeDiscreteInputMat(discr_input_mat, discr_state_mat);

		dyn.pos.state_mat.block(row, 0, 1, 2) = discr_state_mat.block(0, 0, 1, 2);
		dyn.vel.state_mat.block(row, 0, 1, 2) = discr_state_mat.block(1, 0, 1, 2);
		for (int col = 0; col < num_samples - row; col++) {
			dyn.pos.input_mat.block(row + col, col, 1, 2) = discr_input_mat.block(0, 0, 1, 2);
			dyn.vel.input_mat.block(row + col, col, 1, 2) = discr_input_mat.block(1, 0, 1, 2);
		}
	}

	// COP:
	dyn.cop.input_mat.setIdentity();
	dyn.cop.input_mat_tr.setIdentity();
	dyn.cop.input_mat_inv.setIdentity();
	dyn.cop.input_mat_inv_tr.setIdentity();

	// ACCELERATION:
	dyn.acc.state_mat = kGravity / height * (dyn.pos.state_mat - dyn.cop.state_mat);//cop.state_mat should be zero
	dyn.acc.input_mat = kGravity / height * (dyn.pos.input_mat - dyn.cop.input_mat);

}

void DynamicsBuilder::ComputeDiscreteStateMat(Matrix2D &mat, double sample_period) {
	diag_exp_eig_mat_(0, 0) = exp(eigenval_vec_(0) * sample_period);
	diag_exp_eig_mat_(1, 1) = exp(eigenval_vec_(1) * sample_period);

	mat.noalias() = eigenvec_mat_ * diag_exp_eig_mat_ * eigenvec_mat_inv_;
}

void DynamicsBuilder::ComputeDiscreteInputMat(Matrix2D &mat, Matrix2D &discr_state_mat) {
	tmp_mat_.noalias() = -cont_state_mat_inv_ * identity_mat_ * cont_input_vec_;
	mat.noalias() = cont_state_mat_inv_ * discr_state_mat * cont_input_vec_ - tmp_mat_;
}
