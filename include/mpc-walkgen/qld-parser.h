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

	std::vector<double> solution_vec_;
	std::vector<double> hessian_mat_vec_;
	std::vector<double> constr_mat_vec_;
	std::vector<double> constr_vec_;
	std::vector<double> lagr_mult_vec_;

	std::vector<double> war_;
	std::vector<int> iwar_;

    int m_, me_, mmax_, n_, nmax_, mnn_;
    int iout_, ifail_, iprint_, lwar_, liwar_;
    double eps_;


};

}

#endif // MPC_WALKGEN_QLD_PARSER_H
