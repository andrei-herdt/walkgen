#pragma once
#ifndef MPC_WALKGEN_LSSOL_SOLVER_H
#define MPC_WALKGEN_LSSOL_SOLVER_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	lssol-parser.h
///\brief	This class converts the problem t
///
////////////////////////////////////////////////////////////////////////////////

#include <Eigen/Dense>
#include <mpc-walkgen/qp-solver.h>

namespace MPCWalkgen{

  class LSSOLParser:public QPSolver{
  public:
    LSSOLParser(int num_vars, int num_constr);
    virtual ~LSSOLParser();

    virtual void Init();

    virtual void solve(Eigen::VectorXd &qpSolution,
      Eigen::VectorXi &constraints,
      Eigen::VectorXd &initialSolution,
      Eigen::VectorXi &initialConstraints,
      bool warmstart);

    // accessors
    inline Solver name() const
    { return LSSOL; }
    inline bool useCholesky() const
    { return useCholesky_; }
    inline void useCholesky(bool /*ch*/)
    { }

  protected:
    Eigen::VectorXi kx_;
    Eigen::VectorXd bb_;
    Eigen::VectorXd lambda_;
    Eigen::VectorXd bu_;
    Eigen::VectorXd bl_;

    int leniw_;
    int lenw_;
    Eigen::VectorXd war_;
    Eigen::VectorXi iwar_;

    int inform_;
    int iter_;
    double obj_;
    bool useCholesky_;
  };



}

#endif // MPC_WALKGEN_LSSOL_SOLVER_H
