#include <mpc-walkgen/qpoases-parser.h>

#include <iostream>

using namespace MPCWalkgen;
using namespace Eigen;


QPOasesParser::QPOasesParser(SolverData* const parameters, int num_variables, int num_constr)
:QPSolver(parameters, num_variables, num_constr) {
	qp_ = new qpOASES::SQProblem(num_variables, num_constr);
	solution_arr_ = new double[num_variables];
	cstr_init_vec_ = new  qpOASES::Constraints(num_constr);
	bounds_init_vec_ = new  qpOASES::Bounds(num_variables);
}

QPOasesParser::~QPOasesParser() {
	if (qp_ != 0x0) {
		delete qp_;
		qp_ = NULL;
	}
	if (solution_arr_ != 0x0) {
		delete [] solution_arr_;
		solution_arr_ = NULL;
	}
	if (cstr_init_vec_ != 0x0) {
		delete cstr_init_vec_;
		cstr_init_vec_ = NULL;
	}
	if (bounds_init_vec_ != 0x0) {
		delete bounds_init_vec_;
		bounds_init_vec_ = NULL;
	}
}

void QPOasesParser::Init() {
	int max_iterations = 100;

	qp_->init(hessian_mat_().data(), objective_vec_().data(), constr_mat_().data(),
			lv_bounds_vec_().data(), uv_bounds_vec_().data(),
			lc_bounds_vec_().data(), uc_bounds_vec_().data(),
			max_iterations, NULL);
}

void QPOasesParser::Solve(MPCSolution &solution,
		bool warmstart, bool analysis) {

	//qp_->setPrintLevel(qpOASES::PL_NONE);
	qpOASES::Options myoptions;
	//myoptions.setToMPC();
	myoptions.printLevel = qpOASES::PL_LOW;
	myoptions.setToReliable( );
//	myoptions.enableFlippingBounds = qpOASES::BT_FALSE;
//	myoptions.enableRegularisation = qpOASES::BT_FALSE;
//	myoptions.enableNZCTests = qpOASES::BT_FALSE;
//	myoptions.enableEqualities = qpOASES::BT_TRUE;
//	myoptions.initialStatusBounds = qpOASES::ST_INACTIVE;
	qp_->setOptions(myoptions);

	if (warmstart) {
		reorderInitialSolution(solution.initial_solution, solution.init_active_set);
		solution.qp_solution_vec = solution.initial_solution;
		solution.constraints = solution.init_active_set;
		for (int i = 0; i < num_var_; ++i) {
			if (solution.constraints(i) == 0) {//TODO: replace 0,1,-1 by ST_INACTIVE/ST_LOWER/ST_UPPER
				bounds_init_vec_->setupBound(i, qpOASES::ST_INACTIVE);
			} else if (solution.constraints(i) == 1) {
				bounds_init_vec_->setupBound(i, qpOASES::ST_LOWER);
			} else {
				bounds_init_vec_->setupBound(i, qpOASES::ST_UPPER);
			}
		}
		for (int i = num_var_; i < num_var_ + num_constr_; ++i) {
			if (solution.constraints(i) == 0) {
				cstr_init_vec_->setupConstraint(i, qpOASES::ST_INACTIVE);
			} else if (solution.constraints(i) == 1) {
				cstr_init_vec_->setupConstraint(i, qpOASES::ST_LOWER);
			} else {
				cstr_init_vec_->setupConstraint(i, qpOASES::ST_UPPER);
			}
		}
	} else {
		if (solution.qp_solution_vec.rows() != num_var_) {
			solution.qp_solution_vec.setZero(num_var_);
		} else {
			solution.qp_solution_vec.fill(0);
		}
		if (solution.constraints.rows() != num_var_ + num_constr_) {
			solution.constraints.setZero(num_var_ + num_constr_);
		} else {
			solution.constraints.fill(0);
		}
	}

	int num_wsr = parameters_->num_wsrec;
	if (warmstart) {
		qp_->hotstart(hessian_mat_().data(), objective_vec_().data(), constr_mat_().data(),
				lv_bounds_vec_().data(), uv_bounds_vec_().data(),
				lc_bounds_vec_().data(), uc_bounds_vec_().data(),
				num_wsr, NULL);
	} else {
		qp_->hotstart(hessian_mat_().data(), objective_vec_().data(), constr_mat_().data(),
				lv_bounds_vec_().data(), uv_bounds_vec_().data(),
				lc_bounds_vec_().data(), uc_bounds_vec_().data(),
				num_wsr, NULL);
	}

	assert(!qp_->isInfeasible());

	qp_->getPrimalSolution(solution_arr_);
	for (int i = 0; i < num_var_; ++i) {
		solution.qp_solution_vec(i) = solution_arr_[i];
	}

	if (parameters_->analysis) {
		CommonMatrixType obj_val_mat =
				solution.qp_solution_vec.head(num_var_).transpose() * hessian_mat_().block(0, 0, num_var_, num_var_) * solution.qp_solution_vec.head(num_var_)
				+ objective_vec_().head(num_var_).transpose() * solution.qp_solution_vec.head(num_var_);

		solution.analysis.objective_value = obj_val_mat(0, 0);
		solution.analysis.num_iterations = num_wsr;
	}

	/////// TODO: checked until here
	qpOASES::Constraints constr;
	qpOASES::Bounds bounds;
	qp_->getConstraints(constr);
	qp_->getBounds(bounds);
	for (int i=0; i < num_var_; ++i) {
		if (bounds.getStatus(i) == qpOASES::ST_LOWER) {
			solution.constraints(i) = 1;
		} else if (bounds.getStatus(i) == qpOASES::ST_UPPER) {
			solution.constraints(i) = 2;
		} else {
			solution.constraints(i) = 0;
		}
	}
	for (int i = 0; i < num_constr_; ++i) {
		if (constr.getStatus(i) == qpOASES::ST_LOWER) {
			solution.constraints(i + num_var_) = 1;
		} else if (constr.getStatus(i) == qpOASES::ST_UPPER) {
			solution.constraints(i + num_var_) = 2;
		} else {
			solution.constraints(i + num_var_) = 0;
		}
	}

	ReorderIndices(solution.qp_solution_vec, solution.constraints, solution.init_active_set);
}

