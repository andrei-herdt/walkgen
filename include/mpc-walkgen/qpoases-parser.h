#pragma once
#ifndef MPC_WALKGEN_QPOASES_PARSER_H
#define MPC_WALKGEN_QPOASES_PARSER_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	qpoases-parser.h
///\brief	A class to solver the QP problem using qpoases solver
///\author	Herdt Andrei
///\author	Lafaye Jory
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/qp-solver.h>
#include <mpc-walkgen/mpc-debug.h>

# include <qpOASES.hpp>
#include <Eigen/Dense>

//namespace qpOASES{
//  class QProblem;//TODO: ?
//}

namespace MPCWalkgen{

	class QPOasesParser:public QPSolver{
		public:
			QPOasesParser(int nbvar_max, int nbcstr_max);
			virtual ~QPOasesParser();

			virtual void Init();
 
			virtual void Solve(MPCSolution &solutin_data,
					   bool warmstart, bool analysis);

			// accessors
      inline qpOASES::SQProblem *solver() const
      { return qp_; }
			inline SolverName name() const
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

      MPCDebug debug_;

	};

}

#endif // MPC_WALKGEN_QPOASES_PARSER_H
