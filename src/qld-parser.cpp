#include <mpc-walkgen/qld-parser.h>

#include <mpc-walkgen/debug.h>

#include <iostream>

using namespace MPCWalkgen;
using namespace Eigen;


QLDParser::QLDParser(SolverData* const parameters, int num_var_max, int num_constr_max)
:QPSolver(parameters, num_var_max, num_constr_max) {
	solution_vec_.resize(num_var_max);
	std::fill(solution_vec_.begin(), solution_vec_.end(), 0.);
	nmax_ = num_var_max;
	hessian_mat_vec_.resize(nmax_ * nmax_);
	std::fill(hessian_mat_vec_.begin(), hessian_mat_vec_.end(), 0.);
	mmax_ = num_constr_max + 1;
	constr_mat_vec_.resize(nmax_ * mmax_);
	std::fill(constr_mat_vec_.begin(), constr_mat_vec_.end(), 0.);
	constr_vec_.resize(mmax_);
	std::fill(constr_vec_.begin(), constr_vec_.end(), 0.);
	lagr_mult_vec_.resize(num_constr_max + 2*num_var_max);
	std::fill(lagr_mult_vec_.begin(), lagr_mult_vec_.end(), 0.);

	lwar_ = 2*(3*nmax_*nmax_/2 + 10*nmax_ + 2*mmax_ + 20000);
	war_.resize(lwar_);
	std::fill(war_.begin(), war_.end(), 0.);
	liwar_ = num_var_max;
	iwar_.resize(liwar_);
	std::fill(iwar_.begin(), iwar_.end(), 0);
	eps_ = 1e-6;
}

QLDParser::~QLDParser() {

}

void QLDParser::Init() {

}

void QLDParser::Solve(MPCSolution &solution,
		bool warmstart, bool analysis) {

	if (solution.qp_solution_vec.rows() < num_var_) {// TODO: Resize when num_var_max is known
		solution.qp_solution_vec.setZero(num_var_);
	} else {
		solution.qp_solution_vec.fill(0);
	}

	// Transform matrices and constraints vector:
	// Column major required:
	// -------------------------------------------------------
	for (int col = 0; col < num_var_; col++) {
		for (int row = 0; row < num_var_; row++) {
			hessian_mat_vec_[row + nmax_*col] = hessian_mat_(row, col);
		}
	}
	for (int col = 0; col < num_var_; col++) {
		for (int row = 0; row < num_constr_; row++) {
			constr_mat_vec_[row + mmax_*col] = constr_mat_(row, col);
		}
	}
	for (int row = 0; row < num_constr_; row++) {
		constr_vec_[row] = -lc_bounds_vec_(row);
	}

	m_ 			= num_constr_;
	me_ 		= num_eq_constr_;
	n_ 			= num_var_;
	mnn_ 		= m_ + n_ + n_;
	iout_ 		= 0;
	iprint_ 	= 1;
	iwar_[0] 	= 1;

	ql0001_(&m_, &me_, &mmax_, &n_, &nmax_, &mnn_,
			&hessian_mat_vec_[0], objective_vec_().data(),
			&constr_mat_vec_[0], &constr_vec_[0],
			lv_bounds_vec_().data(), uv_bounds_vec_().data(),
			&solution_vec_[0], &lagr_mult_vec_[0], &iout_, &ifail_, &iprint_,
			&war_[0], &lwar_, &iwar_[0], &liwar_, &eps_);

	for (int i = 0; i < num_var_; ++i) {
		solution.qp_solution_vec(i) = solution_vec_[i];
	}


	if (parameters_->analysis) {
		CommonMatrixType obj_val_mat =
				solution.qp_solution_vec.head(num_var_).transpose() * hessian_mat_().block(0, 0, num_var_, num_var_) * solution.qp_solution_vec.head(num_var_)
				+ objective_vec_().head(num_var_).transpose() * solution.qp_solution_vec.head(num_var_);

		solution.analysis.objective_value = obj_val_mat(0, 0);
		if (ifail_ != 0) {
			std::cout << "qld::ifail_: " << ifail_ << std::endl;
		}
		//solution.analysis.num_iterations = num_wsr;
	}

	//ReorderIndices(solution.qp_solution_vec, solution.constraints, solution.init_active_set);
}

