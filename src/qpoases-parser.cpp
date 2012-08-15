#include <mpc-walkgen/qpoases-parser.h>

#include <iostream>
#include <windows.h> 

using namespace MPCWalkgen;
using namespace Eigen;


QPOasesParser::QPOasesParser(int nbvars, int nbcstr)
:QPSolver(nbvars, nbcstr) {
  qp_ = new qpOASES::SQProblem(nbvars, nbcstr);
  solution_vec_ = new double[nbvars];
  cstr_init_vec_ = new  qpOASES::Constraints(nbcstr);
  bounds_init_vec_ = new  qpOASES::Bounds(nbvars);
}

QPOasesParser::~QPOasesParser() {
  if (qp_ != 0x0)
    delete qp_;
  if (solution_vec_ != 0x0)
    delete solution_vec_;
  if (cstr_init_vec_ != 0x0)
    delete cstr_init_vec_;
  if (bounds_init_vec_ != 0x0)
    delete bounds_init_vec_;
}

void QPOasesParser::Init() {

  int max_iterations = 100;

  qp_->init(hessian_mat_().data(), gradient_vec_().data(), cstr_mat_().data(),
    var_l_bounds_vec_().data(), var_u_bounds_vec_().data(),
    constr_l_bounds_vec_().data(), constr_u_bounds_vec_().data(),
    max_iterations, NULL);

}

void QPOasesParser::solve(VectorXd &solution,
                          VectorXi &constraints,
                          VectorXd &initialSolution,
                          VectorXi &initialConstraints,
                          bool warmstart) 
{

  qp_->setPrintLevel(qpOASES::PL_NONE);

  if (warmstart) {
    reorderInitialSolution(initialSolution, initialConstraints);
    solution = initialSolution;
    constraints = initialConstraints;
    for (int i = 0; i < num_vars_; ++i) {
      if (constraints(i) == 0) {//TODO: replace 0,1,-1 by ST_INACTIVE/ST_LOWER/ST_UPPER
        bounds_init_vec_->setupBound(i, qpOASES::ST_INACTIVE);
      }else if (constraints(i) == 1) {
        bounds_init_vec_->setupBound(i, qpOASES::ST_LOWER);
      }else{
        bounds_init_vec_->setupBound(i, qpOASES::ST_UPPER);
      }
    }
    for (int i = num_vars_; i < num_vars_ + num_constr_; ++i) {
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
    if (solution.rows() != num_vars_) {
      solution.setZero(num_vars_);
    } else {
      solution.fill(0);
    }
    if (constraints.rows() != num_vars_ + num_constr_) {
      constraints.setZero(num_vars_ + num_constr_);
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
  int nWSR = 100;
  if (warmstart) {
    qp_->hotstart(hessian_mat_().data(), gradient_vec_().data(), cstr_mat_().data(),
      var_l_bounds_vec_().data(), var_u_bounds_vec_().data(),
      constr_l_bounds_vec_().data(), constr_u_bounds_vec_().data(),
      nWSR, NULL);
  } else {
    qp_->hotstart(hessian_mat_().data(), gradient_vec_().data(), cstr_mat_().data(),
      var_l_bounds_vec_().data(), var_u_bounds_vec_().data(),
      constr_l_bounds_vec_().data(), constr_u_bounds_vec_().data(),
      nWSR, NULL);
  }

  qp_->getPrimalSolution(solution_vec_);
  for (int i = 0; i < num_vars_; ++i) {
    solution(i) = solution_vec_[i];
  }
  qp_->printProperties();

  //2. Messung 
  QueryPerformanceCounter((LARGE_INTEGER*)&g_LastCount); 

  double dTimeDiff = (((double)(g_LastCount-g_CurentCount))/((double)g_Frequency));  

  std::cout << "Zeit: " << dTimeDiff << "at: "  << std::endl<< std::endl<< std::endl<< std::endl; 

  /////// TODO: checked until here
  qpOASES::Constraints ctr;
  qpOASES::Bounds bounds;
  qp_->getConstraints(ctr);
  qp_->getBounds(bounds);
  for (int i=0; i < num_vars_; ++i) {
    if (bounds.getStatus(i) == qpOASES::ST_LOWER) {
      constraints(i) = 1;
    } else if (bounds.getStatus(i) == qpOASES::ST_UPPER) {
      constraints(i) = 2;
    } else {
      constraints(i) = 0;
    }
  }
  for (int i = 0; i < num_constr_; ++i) {
    if (ctr.getStatus(i) == qpOASES::ST_LOWER) {
      constraints(i + num_vars_) = 1;
    }else if (ctr.getStatus(i) == qpOASES::ST_UPPER) {
      constraints(i + num_vars_) = 2;
    }else{
      constraints(i + num_vars_) = 0;
    }
  }

  reorderSolution(solution, constraints, initialConstraints);

  // RESET 
  //bounds_init_vec_->print();

  //cstr_init_vec_->fill(0);

}

