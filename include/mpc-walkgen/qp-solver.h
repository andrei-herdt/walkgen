#pragma once
#ifndef MPC_WALKGEN_QP_SOLVER_H
#define MPC_WALKGEN_QP_SOLVER_H

#include <mpc-walkgen/qp-matrix.h>
#include <mpc-walkgen/qp-vector.h>
#include <mpc-walkgen/types.h>
#include <mpc-walkgen/realclock.h>

namespace MPCWalkgen{

class QPSolver{

	//
	// Public methods:
	//
public:
	QPSolver(const SolverData *parameters, int nbvar_max, int nbcstr_max);
	virtual ~QPSolver() = 0;

	virtual void Init() = 0;

	void Reset();
	virtual void Solve(MPCSolution &solution_data,
			bool warmstart, bool analysis) = 0;

	void DumpProblem(const char *filename);

	void SetVarOrder(const Eigen::VectorXi &order);
	void ctrOrder(const Eigen::VectorXi &order);

	virtual SolverName name() const = 0;

	virtual bool do_build_cholesky() const = 0;
	virtual void useCholesky(bool) = 0;

protected:

	void reorderInitialSolution(CommonVectorType &initialSolution,
			Eigen::VectorXi &initialConstraints);
	void ReorderIndices(CommonVectorType &qp_solution_vec,
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

	int num_variables_, num_variables_max_;
	int num_constr_, num_constr_max_;

	Eigen::VectorXi var_indices_vec_;
	Eigen::VectorXi constr_indices_vec_;

	const SolverData *parameters_;

	RealClock *clock_;

#include <mpc-walkgen/qp-wrapper-inl.h>
};

QPSolver* createQPSolver(const SolverData &parameters, int nbvar_max, int nbcstr_max);
}

#endif // MPC_WALKGEN_QP_SOLVER_H
