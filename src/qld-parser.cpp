#include <mpc-walkgen/qld-parser.h>

#include <mpc-walkgen/debug.h>

#include <iostream>

using namespace MPCWalkgen;
using namespace Eigen;


QLDParser::QLDParser(SolverData* const parameters, int num_var_max, int num_constr_max)
:QPSolver(parameters, num_var_max, num_constr_max) {
	solution_arr_ = new double[num_var_max];
	nmax_ = num_var_max;
	hessian_mat_arr_ = new double[nmax_ * nmax_];
	mmax_ = num_constr_max + 1;
	constr_mat_arr_ = new double[nmax_ * mmax_];
	constr_vec_arr_ = new double[mmax_];
	lagr_mult_arr_ = new double[num_constr_max + 2*num_var_max];

	lwar_ = 2*(3*nmax_*nmax_/2 + 10*nmax_ + 2*mmax_ + 20000);
	war_ = new double[lwar_];
	liwar_ = num_var_max;
	iwar_ = new int[liwar_];
	eps_ = 1e-15;
}

QLDParser::~QLDParser() {
	if (solution_arr_ != 0x0) {
		delete [] solution_arr_;
		solution_arr_ = NULL;
	}
}

void QLDParser::Init() {

}

void QLDParser::Solve(MPCSolution &solution,
		bool warmstart, bool analysis) {

	// Transform matrices and constraints vector:
	// (1) Column major required:
	// (2) First row of constraints matrix and vector are zero
	// -------------------------------------------------------
	for (int col = 0; col < num_var_; col++) {
		for (int row = 0; row < num_var_; row++) {
			hessian_mat_arr_[row + nmax_*col] = hessian_mat_(row, col);
		}
	}
	for (int col = 0; col < num_var_; col++) {
		for (int row = 0; row < num_constr_; row++) {
			constr_mat_arr_[row + mmax_*col] = constr_mat_(row, col);
		}
	}
	for (int row = 0; row < num_constr_; row++) {
		constr_vec_arr_[row] = lc_bounds_vec_(row);
	}

	m_ 			= num_constr_;
	me_ 		= num_eq_constr_;
	n_ 			= num_var_;
	mnn_ 		= m_ + n_ + n_;
	iout_ 		= 0;
	iprint_ 	= 1;
	iwar_[0] 	= 1;

	ql0001_(&m_, &me_, &mmax_, &n_, &nmax_, &mnn_,
			hessian_mat_arr_, objective_vec_().data(),
			constr_mat_arr_, constr_vec_arr_,
			lv_bounds_vec_().data(), uv_bounds_vec_().data(),
			solution_arr_, lagr_mult_arr_, &iout_, &ifail_, &iprint_,
			war_, &lwar_, iwar_, &liwar_, &eps_);

	if (solution.qp_solution_vec.rows() < num_var_) {// TODO: Resize when num_var_max is known
		solution.qp_solution_vec.setZero(num_var_);
	} else {
		solution.qp_solution_vec.fill(0);
	}

	for (int i = 0; i < num_var_; ++i) {
		solution.qp_solution_vec(i) = solution_arr_[i];
	}

	if (parameters_->analysis) {
		//solution.analysis.objective_value = qp_->getObjVal();
		//solution.analysis.num_iterations = num_wsr;
	}

	//ReorderIndices(solution.qp_solution_vec, solution.constraints, solution.init_active_set);
}

