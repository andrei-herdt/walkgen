#include <mpc-walkgen/qp-solver.h>

#include <mpc-walkgen/debug.h>

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
,num_variables_(0)
,num_variables_max_(nbvar_max)
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

void QPSolver::Reset() {
	hessian_mat_.Reset(0.);
	cstr_mat_.Reset(0.);
	gradient_vec_.Reset(0.);
	constr_u_bounds_vec_.Reset(0.);
	constr_l_bounds_vec_.Reset(-0.);
	var_u_bounds_vec_.Reset(0.);
	var_l_bounds_vec_.Reset(-0.);
}

void QPSolver::SetVarIndices(const Eigen::VectorXi &order) {
	var_indices_vec_ = order;
	hessian_mat_.row_indices(order);
	hessian_mat_.column_indices(order);
	cstr_mat_.column_indices(order);
	gradient_vec_.rowOrder(order);
	var_u_bounds_vec_.rowOrder(order);
	var_l_bounds_vec_.rowOrder(order);
}

void QPSolver::SetConstrIndices(const Eigen::VectorXi &order) {
	constr_indices_vec_ = order;
	cstr_mat_.row_indices(order);
	constr_u_bounds_vec_.rowOrder(order);
	constr_l_bounds_vec_.rowOrder(order);
}



void QPSolver::reorderInitialSolution(CommonVectorType &initialSolution,
		VectorXi &initialConstraints) {//TODO: What's the difference betweean those two
	assert(initialSolution.size() >= num_variables_);
	assert(initialConstraints.size() >= num_constr_ + num_variables_);
	CommonVectorType initialSolutionTmp = initialSolution;
	VectorXi initialConstraintsTmp = initialConstraints;
	for (int i = 0; i < num_variables_; ++i) {
		initialSolution(var_indices_vec_(i)) = initialSolutionTmp(i);
		initialConstraints(var_indices_vec_(i)) = initialConstraintsTmp(i);
	}
	for (int i = 0; i < num_constr_; ++i) {
		initialConstraints(constr_indices_vec_(i + num_variables_)) = initialConstraintsTmp(i + num_variables_);
	}

}

void QPSolver::ReorderIndices(CommonVectorType &vec, VectorXi &constraints,
		VectorXi &initialConstraints) {//TODO: What's the difference betweean those two
	//TODO: Simplify this function
	CommonVectorType tmp_vec = vec;
	VectorXi constraintsTmp = constraints;

	for (int i = 0; i < num_variables_; ++i) {
		vec(i) = tmp_vec(var_indices_vec_(i));
		constraints(i) = constraintsTmp(var_indices_vec_(i));
	}
	for (int i = 0; i < num_constr_; ++i) {
		constraints(i + num_variables_) = constraintsTmp(constr_indices_vec_(i + num_variables_));
	}

	initialConstraints = constraints;
}


void QPSolver::DumpProblem(const char *filename, double value, const char *ending) {
	char file_index[256];
	sprintf(file_index, "%.2f", value);
	char name[256];
	strcpy(name, filename); // copy string one into the result.
	strcat(name, file_index);
	strcat(name, ".");
	strcat(name, ending);
	std::ofstream file(name);
	if (file.is_open()) {
		file << "Hessian" << num_variables_ <<","<< num_variables_ <<":" << "\n" << hessian_mat_() << '\n';
		file << "Gradient" << num_variables_ <<":" << "\n" << gradient_vec_() << '\n';
		file << "Constraints matrix" << num_constr_ <<","<< num_variables_ <<":" << "\n" << cstr_mat_() << '\n';
		file << "Constraints upper bounds" << num_constr_ <<":" << "\n" << constr_u_bounds_vec_() << '\n';
		file << "Constraints lower bounds" << num_constr_ <<":" << "\n" << constr_l_bounds_vec_() << '\n';
		file << "Variables upper bounds" << num_variables_ <<":" << "\n" << var_u_bounds_vec_() << '\n';
		file << "Variables lower bounds" << num_variables_ <<":" << "\n" << var_l_bounds_vec_() << '\n';
	}
}

void QPSolver::DumpMatrices(double time, const char *ending) {
	Debug::WriteToFile("H", time, ending, hessian_mat_().block(0, 0, num_variables_, num_variables_));
	Debug::WriteToFile("p", time, ending, gradient_vec_().head(num_variables_));
	Debug::WriteToFile("C", time, ending, cstr_mat_().block(0, 0, num_constr_, num_variables_));
	Debug::WriteToFile("cu", time, ending, constr_u_bounds_vec_().head(num_constr_));
	Debug::WriteToFile("cl", time, ending, constr_l_bounds_vec_().head(num_constr_));
	Debug::WriteToFile("vu", time, ending, var_u_bounds_vec_().head(num_variables_));
	Debug::WriteToFile("vl", time, ending, var_l_bounds_vec_().head(num_variables_));
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
