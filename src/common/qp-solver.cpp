#include "qp-solver.h"

#include <iostream>
#include <fstream> 
#ifdef MPC_WALKGEN_WITH_QPOASES
# include "qp-solvers/qpoases-solver.h"
#endif
#ifdef MPC_WALKGEN_WITH_LSSOL
# include "qp-solvers/lssol-solver.h"
#endif
using namespace MPCWalkgen;
using namespace Eigen;
 
QPSolver::QPSolver(int nbvar_max, int nbcstr_max)
:hessian_mat_(nbvar_max, nbvar_max)
,gradient_vec_(nbvar_max)
,cstr_mat_(nbcstr_max, nbvar_max)
,cstr_arr_(0)
,cstr_u_bounds_vec_(nbcstr_max)
,cstr_l_bounds_vec_(nbcstr_max)
,var_u_bounds_vec_(nbvar_max)
,var_l_bounds_vec_(nbvar_max)
,nbvar_(0)
,nbvar_max_(nbvar_max)
,nbcstr_(0)
,nbcstr_max_(nbcstr_max)
,var_indices_vec_(nbvar_max)
,cstr_indices_vec_(nbvar_max + nbcstr_max) {
  for (int i = 0; i < nbvar_max; ++i) {
    var_indices_vec_(i) = i;
  }
  for (int i = 0; i < nbvar_max + nbcstr_max; ++i) {
    cstr_indices_vec_(i) = i;
  }
  cstr_arr_ = new double[nbvar_max * nbcstr_max];
  hessian_arr_ = new double[nbvar_max * nbvar_max];

}

QPSolver::~QPSolver() {
  if (cstr_arr_ != 0x0)
    delete cstr_arr_;
  if (hessian_arr_ != 0x0)
    delete hessian_arr_;
}

QPMatrix &QPSolver::matrix(const QPMatrixType type) {
  switch(type) {
    case matrixA:
      return cstr_mat_;
    default:
      return hessian_mat_;
  }
}


QPVector &QPSolver::vector(const QPVectorType type) {
  switch(type) {
    case vectorP:
      return gradient_vec_;
    case vectorBU:
      return cstr_u_bounds_vec_;
    case vectorBL:
      return cstr_l_bounds_vec_;
    case vectorXU:
      return var_u_bounds_vec_;
    default:
      return var_l_bounds_vec_;
  }
}

void QPSolver::reset() {
  hessian_mat_.reset();
  cstr_mat_.reset();
  gradient_vec_.reset();
  cstr_u_bounds_vec_.reset();
  cstr_l_bounds_vec_.reset();
  var_u_bounds_vec_.reset();
  var_l_bounds_vec_.reset();
}

void QPSolver::varOrder(const Eigen::VectorXi &order) {
  var_indices_vec_ = order;
  hessian_mat_.rowOrder(order);
  hessian_mat_.colOrder(order);
  cstr_mat_.colOrder(order);
  gradient_vec_.rowOrder(order);
  var_u_bounds_vec_.rowOrder(order);
  var_l_bounds_vec_.rowOrder(order);
}

void QPSolver::ctrOrder(const Eigen::VectorXi &order) {
  cstr_indices_vec_ = order;
  cstr_mat_.rowOrder(order);
  cstr_u_bounds_vec_.rowOrder(order);
  cstr_l_bounds_vec_.rowOrder(order);
}


void QPSolver::reorderInitialSolution(VectorXd &initialSolution,
                                      VectorXi &initialConstraints) 
{
  assert(initialSolution.size() >= nbvar_);
  assert(initialConstraints.size() >= nbcstr_ + nbvar_);
  VectorXd initialSolutionTmp = initialSolution;
  VectorXi initialConstraintsTmp = initialConstraints;
  for (int i = 0; i < nbvar_; ++i) {
    initialSolution(var_indices_vec_(i)) = initialSolutionTmp(i);
    initialConstraints(var_indices_vec_(i)) = initialConstraintsTmp(i);
  }
  for (int i = 0; i < nbcstr_; ++i) {
    initialConstraints(cstr_indices_vec_(i + nbvar_)) = initialConstraintsTmp(i + nbvar_);
  }

}

void QPSolver::reorderSolution(VectorXd &qpSolution, VectorXi &constraints,
                               VectorXi &initialConstraints) {
                                 VectorXd solutionTmp = qpSolution;
                                 VectorXi constraintsTmp = constraints;

                                 for (int i = 0; i < nbvar_; ++i) {
                                   qpSolution(i) = solutionTmp(var_indices_vec_(i));
                                   constraints(i) = constraintsTmp(var_indices_vec_(i));
                                 }
                                 for (int i = 0; i < nbcstr_; ++i) {
                                   constraints(i + nbvar_) = constraintsTmp(cstr_indices_vec_(i + nbvar_));
                                 }

                                 initialConstraints = constraints;
}


void QPSolver::DumpProblem(const char *filename) {  
  std::ofstream file(filename);   
  if (file.is_open()) {     
    file << "Hessian" << hessian_mat_.nbrows() <<","<< hessian_mat_.nbcols()<<":" << "\n" << hessian_mat_() << '\n';
    file << "Gradient" << gradient_vec_.nbrows() <<":" << "\n" << gradient_vec_() << '\n';    
    file << "Constraints Jacobian" << cstr_mat_.nbrows() <<","<< cstr_mat_.nbcols()<<":" << "\n" << cstr_mat_() << '\n';
    file << "Constraints upper bounds" << cstr_u_bounds_vec_.nbrows() <<":" << "\n" << cstr_u_bounds_vec_() << '\n';    
    file << "Constraints lower bounds" << cstr_l_bounds_vec_.nbrows() <<":" << "\n" << cstr_l_bounds_vec_() << '\n';    
    file << "Variables upper bounds" << var_u_bounds_vec_.nbrows() <<":" << "\n" << var_u_bounds_vec_() << '\n';    
    file << "Variables lower bounds" << var_l_bounds_vec_.nbrows() <<":" << "\n" << var_l_bounds_vec_() << '\n';    
  } 
}


QPSolver *MPCWalkgen::createQPSolver(QPSolverType solvertype,
                                     int nbvar, int nbcstr) {
                                       QPSolver *solver = NULL;
#ifdef MPC_WALKGEN_WITH_LSSOL
                                       if (solvertype == QPSOLVERTYPE_LSSOL) {
                                         solver = new LSSOLSolver(nbvarMin, nbCtrMin, nbvarMax, nbCtrMax);
                                       }
#endif //MPC_WALKGEN_WITH_LSSOL

#ifdef MPC_WALKGEN_WITH_QPOASES
                                       if (solvertype == QPOASES) {
                                         solver = new QPOasesSolver(nbvar, nbcstr);
                                       }
#endif //MPC_WALKGEN_WITH_QPOASES

                                       // the user could not ask for an unsupported QPSolverType, as the enum is
                                       // generated
                                       return solver;
}
