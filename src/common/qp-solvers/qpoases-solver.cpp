#include "qpoases-solver.h"
#include <iostream>
#include <windows.h> 

using namespace MPCWalkgen;
using namespace Eigen;


QPOasesSolver::QPOasesSolver(int nbvars, int nbcstr)
:QPSolver(nbvars, nbcstr) {
  qp_ = new qpOASES::SQProblem(nbvars, nbcstr);
  solution_vec_ = new double[nbvars];
  cstr_init_vec_ = new  qpOASES::Constraints(nbcstr);
  bounds_init_vec_ = new  qpOASES::Bounds(nbvars);
}

QPOasesSolver::~QPOasesSolver() {
  if (qp_ != 0x0)
    delete qp_;
  if (solution_vec_ != 0x0)
    delete solution_vec_;
  if (cstr_init_vec_ != 0x0)
    delete cstr_init_vec_;
  if (bounds_init_vec_ != 0x0)
    delete bounds_init_vec_;
}

void QPOasesSolver::Init() {

  int ittMax = 100;

  qp_->init(hessian_mat_().data(), gradient_vec_().data(), cstr_mat_().data(),
    var_l_bounds_vec_().data(), var_u_bounds_vec_().data(),
    cstr_l_bounds_vec_().data(), cstr_u_bounds_vec_().data(),
    ittMax, NULL);

}

void QPOasesSolver::solve(VectorXd &solution,
                          VectorXi &constraints,
                          VectorXd &initialSolution,
                          VectorXi &initialConstraints,
                          bool useWarmStart) 
{

  qp_->setPrintLevel(qpOASES::PL_NONE);

  if (useWarmStart) {
    reorderInitialSolution(initialSolution, initialConstraints);
    solution = initialSolution;
    constraints = initialConstraints;
    for (int i = 0; i < nbvar_; ++i) {
      if (constraints(i) == 0) {//TODO: replace 0,1,-1 by ST_INACTIVE/ST_LOWER/ST_UPPER
        bounds_init_vec_->setupBound(i, qpOASES::ST_INACTIVE);
      }else if (constraints(i) == 1) {
        bounds_init_vec_->setupBound(i, qpOASES::ST_LOWER);
      }else{
        bounds_init_vec_->setupBound(i, qpOASES::ST_UPPER);
      }
    }
    for (int i = nbvar_; i < nbvar_ + nbcstr_; ++i) {
      if (constraints(i) == 0) {
        cstr_init_vec_->setupConstraint(i, qpOASES::ST_INACTIVE);
      } else if (constraints(i) == 1) {
        cstr_init_vec_->setupConstraint(i, qpOASES::ST_LOWER);
      } else {
        cstr_init_vec_->setupConstraint(i, qpOASES::ST_UPPER);
      }
    }

  } else {
    //		cstr_init_vec_->setupAllInactive();
    //		bounds_init_vec_->setupAllFree();
    if (solution.rows() != nbvar_) {
      solution.setZero(nbvar_);
    } else {
      solution.fill(0);
    }
    if (constraints.rows() != nbvar_ + nbcstr_) {
      constraints.setZero(nbvar_ + nbcstr_);
    } else {
      constraints.fill(0);
    }
  }

  //Variablen 
  LONGLONG g_Frequency, g_CurentCount, g_LastCount; 

  //Frequenz holen 
  if (!QueryPerformanceFrequency((LARGE_INTEGER*)&g_Frequency)) 
    std::cout << "Performance Counter nicht vorhanden" << std::endl; 

  //1. Messung 
  QueryPerformanceCounter((LARGE_INTEGER*)&g_CurentCount); 

  //DumpProblem("problem_in_solve.dat");
  //qpOASES::Options myoptions;
  //myoptions.printLevel = qpOASES::PL_HIGH;
  //	qp_->setOptions(myoptions);
  int nWSR = 5;
  qp_->hotstart(hessian_mat_().data(), gradient_vec_().data(), cstr_mat_().data(),
    var_l_bounds_vec_().data(), var_u_bounds_vec_().data(),
    cstr_l_bounds_vec_().data(), cstr_u_bounds_vec_().data(),
    nWSR, NULL);

  qp_->getPrimalSolution(solution_vec_);
  for (int i = 0; i < nbvar_; ++i) {
    solution(i) = solution_vec_[i];
  }

  //2. Messung 
  QueryPerformanceCounter((LARGE_INTEGER*)&g_LastCount); 

  double dTimeDiff = (((double)(g_LastCount-g_CurentCount))/((double)g_Frequency));  

  std::cout << "Zeit: " << dTimeDiff << "at: "  << std::endl<< std::endl<< std::endl<< std::endl; 

  /////// TODO: checked until here
  qpOASES::Constraints ctr;
  qpOASES::Bounds bounds;
  qp_->getConstraints(ctr);
  qp_->getBounds(bounds);
  for(int i=0; i < nbvar_; ++i) {
    if (bounds.getStatus(i) == qpOASES::ST_LOWER) {
      constraints(i) = 1;
    } else if (bounds.getStatus(i) == qpOASES::ST_UPPER) {
      constraints(i) = 2;
    } else {
      constraints(i) = 0;
    }
  }
  for (int i = 0; i < nbcstr_; ++i) {
    if (ctr.getStatus(i) == qpOASES::ST_LOWER) {
      constraints(i + nbvar_) = 1;
    }else if (ctr.getStatus(i) == qpOASES::ST_UPPER) {
      constraints(i + nbvar_) = 2;
    }else{
      constraints(i + nbvar_) = 0;
    }
  }

  reorderSolution(solution, constraints, initialConstraints);

  // RESET 
  //bounds_init_vec_->print();

  //cstr_init_vec_->fill(0);

}

