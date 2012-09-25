#pragma once
#ifndef MPC_WALKGEN_QP_SOLVER_H
#define MPC_WALKGEN_QP_SOLVER_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	qp-solver.hc
///\brief	A class to solver the QP problem
///\author  Andrei Herdt
///\author	Lafaye Jory
///\author      Keith François
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/qp-matrix.h>
#include <mpc-walkgen/qp-vector.h>
#include <mpc-walkgen/types.h>

namespace MPCWalkgen{

class QPSolver{

	//
	// Public methods:
	//
public:
	QPSolver(const SolverData *parameters, int nbvar_max, int nbcstr_max);
	virtual ~QPSolver() = 0;

	virtual void Init() = 0;

	void reset();
	virtual void Solve(MPCSolution &solution_data,
			bool warmstart, bool analysis) = 0;

	void DumpProblem(const char *filename);


	//\name Accessors and mutators
	//\{
	inline QPMatrix &hessian_mat() {return hessian_mat_;}
	inline QPMatrix &constr_mat() {return cstr_mat_;}

	QPVector &vector(const QPVectorType type);

	inline void nbvar (int nbvar)
	{assert(nbvar <= num_vars_max_); num_vars_ = nbvar;}
	inline int nbvar () const {return num_vars_;}
	inline void nbvar_max (int nbvar) {num_vars_max_ = nbvar;}
	inline int nbvar_max () const {return num_vars_max_;}
	inline void num_constr(int num_constr)
	{assert(num_constr <= num_constr_max_); num_constr_ = num_constr;}
	inline int num_constr() const {return num_constr_;}
	inline void nbcstr_max(const int num_constr) {num_constr_max_ = num_constr;}
	inline int nbcstr_max() const {return num_constr_max_;}
	//\}

	void SetVarOrder(const Eigen::VectorXi &order);
	void ctrOrder(const Eigen::VectorXi &order);

	virtual SolverName name() const = 0;

	virtual bool useCholesky() const = 0;
	virtual void useCholesky(bool) = 0;

protected:

	void reorderInitialSolution(CommonVectorType &initialSolution,
			Eigen::VectorXi &initialConstraints);
	void reorderSolution(CommonVectorType &qp_solution_vec,
			Eigen::VectorXi &constraints,
			Eigen::VectorXi &initialConstraints);

protected:
	QPMatrix hessian_mat_;
	double *hessian_arr_;
	QPVector gradient_vec_;

	QPMatrix cstr_mat_;
	double *cstr_arr_;
	QPVector constr_u_bounds_vec_;
	QPVector constr_l_bounds_vec_;

	QPVector var_u_bounds_vec_;
	QPVector var_l_bounds_vec_;

	int num_vars_, num_vars_max_,
	num_constr_, num_constr_max_;

	Eigen::VectorXi var_indices_vec_;
	Eigen::VectorXi constr_indices_vec_;

	const SolverData *parameters_;
};

QPSolver* createQPSolver(const SolverData &parameters, int nbvar_max, int nbcstr_max);
}

#endif // MPC_WALKGEN_QP_SOLVER_H
