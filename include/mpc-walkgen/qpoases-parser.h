#pragma once
#ifndef MPC_WALKGEN_QPOASES_SOLVER_H
#define MPC_WALKGEN_QPOASES_SOLVER_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	qpoases-parser.h
///\brief	A class to solver the QP problem using qpoases solver
///\author	Herdt Andrei
///\author	Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/qp-solver.h>

# include <qpOASES/SQProblem.hpp>
#include <Eigen/Dense>

namespace qpOASES{
  class QProblem;//TODO: ?
}

namespace MPCWalkgen{

	class QPOasesParser:public QPSolver{
		public:
			QPOasesParser(int nbvar_max, int nbcstr_max);
			virtual ~QPOasesParser();

			virtual void Init();

			virtual void solve(Eigen::VectorXd &qpSolution,
					   Eigen::VectorXi &constraints,
					   Eigen::VectorXd &initialSolution,
					   Eigen::VectorXi &initialConstraints,
					   bool warmstart);

			// accessors
			inline Solver name() const
			{ return QPOASES; }
			inline bool useCholesky() const
			{ return false; }
			inline void useCholesky(bool /*ch*/)
			{}

		protected:
			qpOASES::SQProblem *qp_;
			double *solution_vec_;
			qpOASES::Constraints *cstr_init_vec_;
			qpOASES::Bounds *bounds_init_vec_;

	};

}

#endif // MPC_WALKGEN_QPOASES_SOLVER_H
