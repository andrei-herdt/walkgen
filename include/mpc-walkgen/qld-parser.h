#pragma once
#ifndef MPC_WALKGEN_QLD_PARSER_H
#define MPC_WALKGEN_QLD_PARSER_H

#include <mpc-walkgen/qp-solver.h>

#include <qld.h>
#include <Eigen/Dense>

namespace MPCWalkgen{

class QLDParser : public QPSolver{
public:
	QLDParser(SolverData* const paremeters, int num_var_max, int num_constr_max);
	virtual ~QLDParser();

	virtual void Init();

	virtual void Solve(MPCSolution &solutin_data,
			bool warmstart, bool analysis);

	// accessors
	inline SolverName name() const
	{ return QLD; }
	inline bool do_build_cholesky() const
	{ return false; }
	inline void useCholesky(bool /*ch*/)
	{}

protected:

	double *constr_vec_arr_;
	double *lagr_mult_arr_;
	double *war_;
	int *iwar_;
    int m_, me_, mmax_, n_, nmax_, mnn_;
    int iout_, ifail_, iprint_, lwar_, liwar_;
    double eps_;

	double *solution_arr_;

};

}

#endif // MPC_WALKGEN_QLD_PARSER_H
