#pragma once
#ifndef MPC_WALKGEN_QPOASES_PARSER_H
#define MPC_WALKGEN_QPOASES_PARSER_H

#include <mpc-walkgen/qp-solver.h>

#include <qpOASES.hpp>
#include <Eigen/Dense>

namespace MPCWalkgen{

class QPOasesParser : public QPSolver{
public:
	QPOasesParser(SolverData* const paremeters, int nbvar_max, int nbcstr_max);
	virtual ~QPOasesParser();

	virtual void Init();

	virtual void Solve(MPCSolution &solutin_data,
			bool warmstart, bool analysis);

	// accessors
	inline qpOASES::SQProblem *solver() const
	{ return qp_; }
	inline SolverName name() const
	{ return QPOASES; }
	inline bool do_build_cholesky() const
	{ return false; }
	inline void useCholesky(bool /*ch*/)
	{}

protected:
	qpOASES::SQProblem *qp_;
	double *solution_arr_;
	qpOASES::Constraints *cstr_init_vec_;
	qpOASES::Bounds *bounds_init_vec_;

};

}

#endif // MPC_WALKGEN_QPOASES_PARSER_H
