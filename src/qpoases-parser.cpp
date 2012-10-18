#include <mpc-walkgen/qpoases-parser.h>

#include <iostream>

using namespace MPCWalkgen;
using namespace Eigen;


QPOasesParser::QPOasesParser(const SolverData *parameters, int num_variables, int num_constr)
:QPSolver(parameters, num_variables, num_constr) {
  qp_ = new qpOASES::SQProblem(num_variables, num_constr);
  solution_vec_ = new double[num_variables];
  cstr_init_vec_ = new  qpOASES::Constraints(num_constr);
  bounds_init_vec_ = new  qpOASES::Bounds(num_variables);
}

QPOasesParser::~QPOasesParser() {
  if (qp_ != 0x0) {
    delete qp_;
    qp_ = NULL;
  }
  if (solution_vec_ != 0x0)
    delete [] solution_vec_;
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

void QPOasesParser::Solve(MPCSolution &solution_data,
                          bool warmstart, bool analysis) 
{

  qp_->setPrintLevel(qpOASES::PL_NONE);

  if (warmstart) {
    reorderInitialSolution(solution_data.initialSolution, solution_data.initialConstraints);
    solution_data.qp_solution_vec = solution_data.initialSolution;
    solution_data.constraints = solution_data.initialConstraints;
    for (int i = 0; i < num_variables_; ++i) {
      if (solution_data.constraints(i) == 0) {//TODO: replace 0,1,-1 by ST_INACTIVE/ST_LOWER/ST_UPPER
        bounds_init_vec_->setupBound(i, qpOASES::ST_INACTIVE);
      }else if (solution_data.constraints(i) == 1) {
        bounds_init_vec_->setupBound(i, qpOASES::ST_LOWER);
      }else{
        bounds_init_vec_->setupBound(i, qpOASES::ST_UPPER);
      }
    }
    for (int i = num_variables_; i < num_variables_ + num_constr_; ++i) {
      if (solution_data.constraints(i) == 0) {
        cstr_init_vec_->setupConstraint(i, qpOASES::ST_INACTIVE);
      } else if (solution_data.constraints(i) == 1) {
        cstr_init_vec_->setupConstraint(i, qpOASES::ST_LOWER);
      } else {
        cstr_init_vec_->setupConstraint(i, qpOASES::ST_UPPER);
      }
    }

  } else {
    if (solution_data.qp_solution_vec.rows() != num_variables_) {
      solution_data.qp_solution_vec.setZero(num_variables_);
    } else {
      solution_data.qp_solution_vec.fill(0);
    }
    if (solution_data.constraints.rows() != num_variables_ + num_constr_) {
      solution_data.constraints.setZero(num_variables_ + num_constr_);
    } else {
      solution_data.constraints.fill(0);
    }
  }


  int num_wsr = parameters_->num_wsrec;
  if (warmstart) {
    qp_->hotstart(hessian_mat_().data(), gradient_vec_().data(), cstr_mat_().data(),
      var_l_bounds_vec_().data(), var_u_bounds_vec_().data(),
      constr_l_bounds_vec_().data(), constr_u_bounds_vec_().data(),
      num_wsr, NULL);
  } else {
    qp_->hotstart(hessian_mat_().data(), gradient_vec_().data(), cstr_mat_().data(),
      var_l_bounds_vec_().data(), var_u_bounds_vec_().data(),
      constr_l_bounds_vec_().data(), constr_u_bounds_vec_().data(),
      num_wsr, NULL);
  }

  qp_->getPrimalSolution(solution_vec_);
  for (int i = 0; i < num_variables_; ++i) {
    solution_data.qp_solution_vec(i) = solution_vec_[i];
  }

  /////// TODO: checked until here
  qpOASES::Constraints ctr;
  qpOASES::Bounds bounds;
  qp_->getConstraints(ctr);
  qp_->getBounds(bounds);
  for (int i=0; i < num_variables_; ++i) {
    if (bounds.getStatus(i) == qpOASES::ST_LOWER) {
      solution_data.constraints(i) = 1;
    } else if (bounds.getStatus(i) == qpOASES::ST_UPPER) {
      solution_data.constraints(i) = 2;
    } else {
      solution_data.constraints(i) = 0;
    }
  }
  for (int i = 0; i < num_constr_; ++i) {
    if (ctr.getStatus(i) == qpOASES::ST_LOWER) {
      solution_data.constraints(i + num_variables_) = 1;
    }else if (ctr.getStatus(i) == qpOASES::ST_UPPER) {
      solution_data.constraints(i + num_variables_) = 2;
    }else{
      solution_data.constraints(i + num_variables_) = 0;
    }
  }

  reorderSolution(solution_data.qp_solution_vec, solution_data.constraints, solution_data.initialConstraints);
}

