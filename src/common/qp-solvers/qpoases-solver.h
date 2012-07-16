#pragma once
#ifndef MPC_WALKGEN_QPOASES_SOLVER_H
#define MPC_WALKGEN_QPOASES_SOLVER_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	qpoases-solver.h
///\brief	A class to solver the QP problem using qpoases solver
///\author	Herdt Andrei
///\author	Lafaye Jory
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////


#include <Eigen/Dense>
#include "../qp-solver.h"
#ifdef USE_QPOASES_3_0
# include <qpOASES/SQProblem.hpp>
#else //
# include <QProblem.hpp>
#endif //USE_QPOASES_3_0


namespace qpOASES{
  class QProblem;
}
namespace MPCWalkgen{

	class QPOasesSolver:public QPSolver{
		public:
			QPOasesSolver(const int nbvars = QPSolver::DefaultNbVars_, 
				const int nbcstr = QPSolver::DefaultNbCstr_);
			virtual ~QPOasesSolver();

			virtual void Init();

			// accessors
			inline QPSolverType getType() const
			{ return QPSOLVERTYPE_QPOASES; }
			inline bool useCholesky() const
			{ return false; }
			inline void useCholesky(bool /*ch*/)
			{}

			virtual void solve(Eigen::VectorXd &qpSolution,
					   Eigen::VectorXi &constraints,
					   Eigen::VectorXd &initialSolution,
					   Eigen::VectorXi &initialConstraints,
					   bool useWarmStart);

		protected:
			virtual bool resizeAll();

		protected:
			qpOASES::SQProblem *qp_;
			double *solution_vec_;
			qpOASES::Constraints *cstr_init_vec_;
			qpOASES::Bounds *bounds_init_vec_;

	};

}

/*!
* \class MPCWalkgen::QPOasesLSolver qpoases-solver.h "Definition"
* \ingroup solver
* \brief QPSolver based on the qpoases library
*/

#endif // MPC_WALKGEN_QPOASES_SOLVER_H
