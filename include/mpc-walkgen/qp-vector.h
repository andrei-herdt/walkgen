#pragma once
#ifndef MPC_WALKGEN_QP_VECTOR_H
#define MPC_WALKGEN_QP_VECTOR_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	qp-vector.h
///\brief	A class to store QP matrices
///\author Andrei Herdt
///\author	Lafaye Jory
///\author      Keith François
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////

#include <vector>
#include <Eigen/Dense>

namespace MPCWalkgen{

  class QPVector{
  public:
    QPVector(const int num_rows);
    ~QPVector();

    void addTerm(const Eigen::VectorXd &vec, const int row);
    void addTerm(double value, int first_row);

    void setConstantPart(const Eigen::VectorXd &vec);

    void reset();

    void resize(const int num_rows);

    void rowOrder(const Eigen::VectorXi &order);

    inline Eigen::VectorXd &operator()(void) {
      return vector_;
    }
    inline double &operator()(int row){return vector_(row);}

    inline int num_rows() const	 {return num_rows_;}

  private:

    Eigen::VectorXd constantPart_, vector_;

    int num_rows_;

    Eigen::VectorXi index_order_;
  };
}

#endif // MPC_WALKGEN_QP_VECTOR_H
