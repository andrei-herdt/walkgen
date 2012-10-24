#pragma once
#ifndef MPC_WALKGEN_LSSOL_SOLVER_H
#define MPC_WALKGEN_LSSOL_SOLVER_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	lssol-parser.h
///\brief	This class converts the problem t
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/qp-solver.h>

#include <Eigen/Dense>

namespace MPCWalkgen{

  class LSSOLParser:public QPSolver{
  public:
    LSSOLParser(const SolverData *parameters, int num_vars, int num_constr);
    virtual ~LSSOLParser();

    virtual void Init();

    virtual void Solve(MPCSolution &solution_data,
      bool warmstart, bool analyze);

    // accessors
    inline SolverName name() const
    { return LSSOL; }
    inline bool do_build_cholesky() const
    { return useCholesky_; }
    inline void useCholesky(bool /*ch*/)
    { }

  protected:
    Eigen::VectorXi kx_;
    CommonVectorType bb_;
    CommonVectorType lambda_;
    CommonVectorType bu_;
    CommonVectorType bl_;

    int leniw_;
    int lenw_;
    CommonVectorType war_;
    Eigen::VectorXi iwar_;

    int inform_;
    int iter_;
    double obj_;
    bool useCholesky_;
  };



}

#endif // MPC_WALKGEN_LSSOL_SOLVER_H
