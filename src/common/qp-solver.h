#pragma once
#ifndef MPC_WALKGEN_QP_SOLVER_H
#define MPC_WALKGEN_QP_SOLVER_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	qp-solver.h
///\brief	A class to solver the QP problem
///\author  Andrei Herdt
///\author	Lafaye Jory
///\author      Keith François
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////


#include <Eigen/Dense>
#include "qp-matrix.h"
#include "qp-vector.h"
#include "types.h"

namespace MPCWalkgen{

  class QPSolver{
  public:
    static const int DefaultNbVars_;
    static const int DefaultNbCstr_;

  public:
    QPSolver(const int nbvars = DefaultNbVars_, const int nbcstr = DefaultNbCstr_);
    virtual ~QPSolver() = 0;

    virtual void Init() = 0;

    void reset();
    virtual void solve(Eigen::VectorXd & qpSolution,
      Eigen::VectorXi & constraints,
      Eigen::VectorXd & initialSolution,
      Eigen::VectorXi & initialConstraints,
      bool useWarmStart) = 0;

    void dump();
    void DumpProblem(const char *filename); 

  public:
    QPMatrix & matrix(const QPMatrixType type);
    QPVector & vector(const QPVectorType type);

    void nbVar(const int nbvars);
    inline int nbVar() const {return nbvars_;}
    void nbCtr(const int nbcstr);
    inline int nbCtr() const {return nbcstr_;}
    void addNbCtr(const int addCtr);

    void varOrder(const Eigen::VectorXi & order);
    void ctrOrder(const Eigen::VectorXi & order);

    virtual QPSolverType getType() const =0;

    // can we / should we use the cholesly matrix
    virtual bool useCholesky() const =0;
    virtual void useCholesky(bool)=0;

  protected:
    virtual bool resizeAll();

    void reorderInitialSolution(Eigen::VectorXd & initialSolution,
      Eigen::VectorXi & initialConstraints);
    void reorderSolution(Eigen::VectorXd & qpSolution,
      Eigen::VectorXi & constraints,
      Eigen::VectorXi & initialConstraints);

  protected:
    QPMatrix hessian_mat_;
    double *hessian_arr_;
    QPVector gradient_vec_;

    QPMatrix cstr_mat_;
    double *cstr_arr_;
    QPVector cstr_u_bounds_vec_;
    QPVector cstr_l_bounds_vec_;

    QPVector var_u_bounds_vec_;
    QPVector var_l_bounds_vec_;

    int nbvars_;
    int nbcstr_;

    Eigen::VectorXi var_indices_vec_;
    Eigen::VectorXi cstr_indices_vec_;
  };
  QPSolver* createQPSolver(QPSolverType solvertype,
    int nbvars = QPSolver::DefaultNbVars_,
    int nbcstr = QPSolver::DefaultNbCstr_);
}


/*! @defgroup solver solvers
*  @ingroup private
* this group gathers the solver available with mpc-walkgen
*/

/*!
* \class MPCWalkgen::QPSolver qp-solver.h "Definition"
* \ingroup solver
* \brief Abstract interface of the solver used
*/

/*! \fn MPCWalkgen::QPSolver::QPSolver(const int nbVarMax=DefaultNbVars_, const int nbCtrMax=DefaultNbCstr_)
* \brief Constructor
* \param nbVarMax Maximum number of variables
* \param nbCtrMax Maximum number of constraints
*/

/*! \fn QPMatrix & MPCWalkgen::QPSolver::matrix(const QPMatrixType type)
* \brief Return the desired QP matrix
*/

/*! \fn MPCWalkgen::QPSolver::matrix(const QPMatrixType type);
* \brief return a QPMatrix.
*/

/*! \fn MPCWalkgen::QPSolver::reset(const bool withConstantPart = true)
* \brief Erase the problem
* \param withConstantPart if true, constant part of QPMatrices will replace current values
*/

/*! \fn MPCWalkgen::QPSolver::nbVar(const int nbVar)
* \brief Setter to modify the number of variables
*/

/*! \fn MPCWalkgen::QPSolver::nbCtr(const int nbCtr)
* \brief Setter to modify the number of constraints
*/

/*! \fn MPCWalkgen::QPSolver::addNbCtr(const int addCtr)
* \brief Augment the number of constraints
*/

/*! \fn Solver MPCWalkgen::QPSolver::getType() const
* \brief Return the type of solver
*/

/*! \fn bool MPCWalkgen::QPSolver::useCholesky() const
* \brief Return true is the solver makes use of the cholesky matrix
*/

/*! \fn void MPCWalkgen::QPSolver::useCholesky(bool)
* \brief indicates if the solver can use of the cholesky matrix (unavailable for some solvers)
*/

/*! \fn MPCWalkgen::QPSolver::addNbCtr(const int addCtr)
* \brief Augment the number of constraints
*/

/*! \fn MPCWalkgen::QPSolver::solve(MPCSolution & solution)
* \brief Solve the problem defined by this form : \f$ \left\{
* \begin{array}{l}
* 	min \| x^t Q x + p^t x \|\\
* 	bl \leq Ax \leq bu, \\
* 	xl \leq x \leq xu \\
* \end{array}
* \right.
* \f$
*/
#endif // MPC_WALKGEN_QP_SOLVER_H
