#pragma once
#ifndef MPC_WALKGEN_QP_MATRIX_H
#define MPC_WALKGEN_QP_MATRIX_H

#include <mpc-walkgen/types.h>

#include <vector>

namespace MPCWalkgen{

  class QPMatrix{
    //
    // Public methods:
    //
  public:
    QPMatrix(const int num_rows, const int num_cols);

    ~QPMatrix();

    void AddTerm(const EigenMatrixXdRM &mat,
      const int first_row, const int first_col);

    void setConstantPart(const CommonMatrixType &mat);

    void reset();

    void resize(const int num_rows, const int num_cols);

    CommonMatrixType &cholesky();
    CommonMatrixType &cholesky(CommonMatrixType &partialCholesky);

    void colOrder(const Eigen::VectorXi &order);
    void rowOrder(const Eigen::VectorXi &order);

    // \name Accessors and mutators
    // \{
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
    // \}

    //
    // Private methods:
    //
  private:
    void BuildCholesky(const CommonMatrixType &partialCholesky);
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
