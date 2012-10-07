#include <mpc-walkgen/qp-solver.h>

#include <iostream>
#include <fstream> 

using namespace MPCWalkgen;
using namespace Eigen;

QPSolver::QPSolver(const SolverData *parameters, int nbvar_max, int nbcstr_max)
:hessian_mat_(nbvar_max, nbvar_max)
,hessian_arr_(NULL)
,gradient_vec_(nbvar_max)
,cstr_mat_(nbcstr_max, nbvar_max)
,cstr_arr_(NULL)
,constr_u_bounds_vec_(nbcstr_max)
,constr_l_bounds_vec_(nbcstr_max)
,var_u_bounds_vec_(nbvar_max)
,var_l_bounds_vec_(nbvar_max)
,num_vars_(0)
,num_vars_max_(nbvar_max)
,num_constr_(0)
,num_constr_max_(nbcstr_max)
,var_indices_vec_(nbvar_max)
,constr_indices_vec_(nbvar_max + nbcstr_max)
,parameters_(parameters)
,clock_(NULL) {
	for (int i = 0; i < nbvar_max; ++i) {
		var_indices_vec_(i) = i;
	}
	for (int i = 0; i < nbvar_max + nbcstr_max; ++i) {
		constr_indices_vec_(i) = i;
	}
	cstr_arr_ = new double[nbvar_max * nbcstr_max];
	hessian_arr_ = new double[nbvar_max * nbvar_max];

}

QPSolver::~QPSolver() {
	if (cstr_arr_ != 0x0) {
		delete[] cstr_arr_;
		cstr_arr_ = NULL;
	}
	if (hessian_arr_ != 0x0) {
		delete[] hessian_arr_;
		hessian_arr_ = NULL;
	}
}

QPVector &QPSolver::vector(const QPVectorType type) {//TODO:Remove this
	switch(type) {
	case vectorP:
		return gradient_vec_;
	case vectorBU:
		return constr_u_bounds_vec_;
	case vectorBL:
		return constr_l_bounds_vec_;
	case vectorXU:
		return var_u_bounds_vec_;
	default:
		return var_l_bounds_vec_;
	}
}

void QPSolver::reset() {
	hessian_mat_.reset();
	cstr_mat_.reset();
	gradient_vec_.reset();
	constr_u_bounds_vec_.reset();
	constr_l_bounds_vec_.reset();
	var_u_bounds_vec_.reset();
	var_l_bounds_vec_.reset();
}

void QPSolver::SetVarOrder(const Eigen::VectorXi &order) {
	var_indices_vec_ = order;
	hessian_mat_.rowOrder(order);
	hessian_mat_.colOrder(order);
	cstr_mat_.colOrder(order);
	gradient_vec_.rowOrder(order);
	var_u_bounds_vec_.rowOrder(order);
	var_l_bounds_vec_.rowOrder(order);
}

void QPSolver::ctrOrder(const Eigen::VectorXi &order) {
	constr_indices_vec_ = order;
	cstr_mat_.rowOrder(order);
	constr_u_bounds_vec_.rowOrder(order);
	constr_l_bounds_vec_.rowOrder(order);
}



void QPSolver::reorderInitialSolution(CommonVectorType &initialSolution,
		VectorXi &initialConstraints) {
	assert(initialSolution.size() >= num_vars_);
	assert(initialConstraints.size() >= num_constr_ + num_vars_);
	CommonVectorType initialSolutionTmp = initialSolution;
	VectorXi initialConstraintsTmp = initialConstraints;
	for (int i = 0; i < num_vars_; ++i) {
		initialSolution(var_indices_vec_(i)) = initialSolutionTmp(i);
		initialConstraints(var_indices_vec_(i)) = initialConstraintsTmp(i);
	}
	for (int i = 0; i < num_constr_; ++i) {
		initialConstraints(constr_indices_vec_(i + num_vars_)) = initialConstraintsTmp(i + num_vars_);
	}

}

void QPSolver::reorderSolution(CommonVectorType &qp_solution_vec, VectorXi &constraints,
		VectorXi &initialConstraints)
{
	CommonVectorType solutionTmp = qp_solution_vec;
	VectorXi constraintsTmp = constraints;

	for (int i = 0; i < num_vars_; ++i) {
		qp_solution_vec(i) = solutionTmp(var_indices_vec_(i));
		constraints(i) = constraintsTmp(var_indices_vec_(i));
	}
	for (int i = 0; i < num_constr_; ++i) {
		constraints(i + num_vars_) = constraintsTmp(constr_indices_vec_(i + num_vars_));
	}

	initialConstraints = constraints;
}


void QPSolver::DumpProblem(const char *filename) {  
	std::ofstream file(filename);
	if (file.is_open()) {
		file << "Hessian" << hessian_mat_.num_rows() <<","<< hessian_mat_.num_cols()<<":" << "\n" << hessian_mat_() << '\n';
		file << "Gradient" << gradient_vec_.num_rows() <<":" << "\n" << gradient_vec_() << '\n';
		file << "Constraints Jacobian" << cstr_mat_.num_rows() <<","<< cstr_mat_.num_cols()<<":" << "\n" << cstr_mat_() << '\n';
		file << "Constraints upper bounds" << constr_u_bounds_vec_.num_rows() <<":" << "\n" << constr_u_bounds_vec_() << '\n';
		file << "Constraints lower bounds" << constr_l_bounds_vec_.num_rows() <<":" << "\n" << constr_l_bounds_vec_() << '\n';
		file << "Variables upper bounds" << var_u_bounds_vec_.num_rows() <<":" << "\n" << var_u_bounds_vec_() << '\n';
		file << "Variables lower bounds" << var_l_bounds_vec_.num_rows() <<":" << "\n" << var_l_bounds_vec_() << '\n';
	}
}

#include <mpc-walkgen/qpoases-parser.h>
#include <mpc-walkgen/lssol-parser.h>

QPSolver *MPCWalkgen::createQPSolver(const SolverData &parameters, int num_vars, int num_constr) 
{//TODO: Remove this
	QPSolver *solver = NULL;
	if (parameters.name == LSSOL) {
		solver = new LSSOLParser(&parameters, num_vars, num_constr);
	} else if (parameters.name == QPOASES) {
		solver = new QPOasesParser(&parameters, num_vars, num_constr);
	}
	return solver;
}
