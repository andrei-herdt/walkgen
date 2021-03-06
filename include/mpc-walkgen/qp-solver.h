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
	QPSolver(SolverData* const parameters, int num_var_max, int num_constr_max);
	virtual ~QPSolver() = 0;

	virtual void Init() = 0;

	void Reset();

	virtual void Solve(MPCSolution &solution_data,
			bool warmstart,
			bool analysis
	) = 0;

	void DumpProblem(const char *filename,
			double value,
			const char *ending = "dat"
	);

	void DumpMatrices(double time,
			const char *ending = "dat"
	);


	void SetVarIndices(const Eigen::VectorXi &order
	);
	void SetConstrIndices(const Eigen::VectorXi &order
	);

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
	double *hessian_mat_arr_;
	QPVector objective_vec_;

	QPMatrix constr_mat_;
	double *constr_mat_arr_;
	QPVector uc_bounds_vec_;
	QPVector lc_bounds_vec_;

	QPVector uv_bounds_vec_;
	QPVector lv_bounds_vec_;

	int num_var_, num_var_max_;
	int num_constr_, num_constr_max_;
	int num_eq_constr_;

	Eigen::VectorXi var_indices_vec_;
	Eigen::VectorXi constr_indices_vec_;

	SolverData* const parameters_;

	RealClock *clock_;

#include <mpc-walkgen/qp-wrapper-inl.h>
};

QPSolver* createQPSolver(SolverData &parameters, int nbvar_max, int nbcstr_max);
}

#endif // MPC_WALKGEN_QP_SOLVER_H
