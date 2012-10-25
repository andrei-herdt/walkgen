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

#include <mpc-walkgen/types.h>

#include <vector>

namespace MPCWalkgen{

  class QPVector{
  public:
    QPVector(const int num_rows);
    ~QPVector();

    void Set(const CommonVectorType &vec, const int row);
    void Set(double value, int row);
    void Add(const CommonVectorType &vec, const int row);
    void Add(double value, int row);

    void setConstantPart(const CommonVectorType &vec);

    void Reset(double value);

    void resize(const int num_rows);

    void rowOrder(const Eigen::VectorXi &order);

    inline CommonVectorType &operator()(void) {
      return vector_;
    }
    inline double &operator()(int row){return vector_(row);}

    inline int num_rows() const	 {return num_rows_;}

  private:

    CommonVectorType constantPart_, vector_;

    int num_rows_;

    Eigen::VectorXi index_order_;
  };
}

#endif // MPC_WALKGEN_QP_VECTOR_H
