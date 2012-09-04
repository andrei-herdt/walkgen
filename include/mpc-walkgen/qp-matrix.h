#pragma once
#ifndef MPC_WALKGEN_QP_MATRIX_H
#define MPC_WALKGEN_QP_MATRIX_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	qp-matrix.h
///\brief	A class to store QP matrices
///\author  Herdt Andrei
///\author	Lafaye Jory
///\author      Keith François
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/types.h>

#include <vector>
#include <Eigen/Dense>

namespace MPCWalkgen{

  class QPMatrix{
    //
    // Public methods:
    //
  public:
    QPMatrix(const int num_rows, const int num_cols);

    ~QPMatrix();

    void addTerm(const EigenMatrixXdRM &mat,
      const int first_row, const int first_col);

    void setTerm(const CommonMatrixType &mat,
      const int first_row, const int first_col);

    void setConstantPart(const CommonMatrixType &mat);

    void reset();

    void resize(const int num_rows, const int num_cols);

    CommonMatrixType &cholesky();
    CommonMatrixType &cholesky(CommonMatrixType &partialCholesky);

    void colOrder(const Eigen::VectorXi &order);
    void rowOrder(const Eigen::VectorXi &order);
    void relative_row_order(const Eigen::VectorXi &order);
    void relative_col_order(const Eigen::VectorXi &order);


    // accessors
    inline CommonMatrixType &operator()(void) {
      cholesky_old_mat_ = true;
      return matrix_;
    }
    inline const CommonMatrixType &operator()(void) const {
      return matrix_;
    }

    inline double &operator()(int row, int col=0){return matrix_(row,col);}

    inline int num_rows() const	 {return num_rows_;}
    inline int num_cols() const    {return num_cols_;}

    //
    // Private methods:
    //
  private:
    void computeCholesky(const CommonMatrixType &partialCholesky);
    void computeCholesky();

    //
    // Private data members:
    //
  private:

    CommonMatrixType constant_mat_;
    CommonMatrixType matrix_;
    CommonMatrixType cholesky_mat_;

    int num_rows_;
    int num_cols_;

    bool cholesky_old_mat_;

    Eigen::VectorXi row_indices_vec_;
    Eigen::VectorXi col_indices_vec_;

  };

}
#endif // MPC_WALKGEN_QP_MATRIX_H
