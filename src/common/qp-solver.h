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
    QPSolver(int nbvar_max, int nbcstr_max);
    virtual ~QPSolver() = 0;

    virtual void Init() = 0;

    void reset();
    virtual void solve(Eigen::VectorXd & qpSolution,
      Eigen::VectorXi & constraints,
      Eigen::VectorXd & initialSolution,
      Eigen::VectorXi & initialConstraints,
      bool useWarmStart) = 0;

    void DumpProblem(const char *filename); 
 
  public:
    QPMatrix & matrix(const QPMatrixType type);
    QPVector & vector(const QPVectorType type);

    inline void nbvar (int nbvar) 
    {assert(nbvar <= nbvar_max_); nbvar_ = nbvar;}
    inline int nbvar () const {return nbvar_;}
    inline void nbvar_max (int nbvar) {nbvar_max_ = nbvar;}
    inline int nbvar_max () const {return nbvar_max_;}
    inline void nbcstr(int nbcstr) 
    {assert(nbcstr <= nbcstr_max_); nbcstr_ = nbcstr;}
    inline int nbcstr() const {return nbcstr_;}
    inline void nbcstr_max(const int nbcstr) {nbcstr_max_ = nbcstr;}
    inline int nbcstr_max() const {return nbcstr_max_;}

    void varOrder(const Eigen::VectorXi & order);
    void ctrOrder(const Eigen::VectorXi & order);

    virtual QPSolverType getType() const =0;

    virtual bool useCholesky() const =0;
    virtual void useCholesky(bool)=0;

  protected:

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

    int nbvar_, nbvar_max_,
      nbcstr_, nbcstr_max_;

    Eigen::VectorXi var_indices_vec_;
    Eigen::VectorXi cstr_indices_vec_;
  };

  QPSolver* createQPSolver(QPSolverType solvertype, int nbvar_max, int nbcstr_max);
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

/*! \fn MPCWalkgen::QPSolver::QPSolver(const int nbvarMax=DefaultNbVars_, const int nbCtrMax=DefaultNbCstr_)
* \brief Constructor
* \param nbvarMax Maximum number of variables
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

/*! \fn MPCWalkgen::QPSolver::nbvar(const int nbvar)
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
