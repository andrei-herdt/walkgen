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
  public:

    /// \name Typedefs
    /// \{
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> EigenMatrixXdRM;
    /// \}

    QPMatrix(const int nbrows = 0, const int nbcols = 0);

    ~QPMatrix();

    void addTerm(const Eigen::MatrixXd &mat,
      const int row = 0, const int col = 0);

    void setTerm(const Eigen::MatrixXd &mat,
      const int row = 0, const int col = 0);

    void setConstantPart(const Eigen::MatrixXd &mat);

    void reset();

    void resize(const int nbrows, const int nbcols);

    Eigen::MatrixXd &cholesky();
    Eigen::MatrixXd &cholesky(Eigen::MatrixXd &partialCholesky);

    void colOrder(const Eigen::VectorXi &order);
    void rowOrder(const Eigen::VectorXi &order);


    // accessors
    inline EigenMatrixXdRM &operator()(void) {
      cholesky_old_mat_ = true;
      return matrix_;
    }
    inline const EigenMatrixXdRM &operator()(void) const {
      return matrix_;
    }

    inline double &operator()(int row, int col=0){return matrix_(row,col);}

    inline int nbrows() const	 {return nbrows_;}
    inline int nbcols() const    {return nbcols_;}

  private:
    void computeCholesky(const Eigen::MatrixXd &partialCholesky);
    void computeCholesky();

  private:

    EigenMatrixXdRM constant_mat_;
    EigenMatrixXdRM matrix_;
    Eigen::MatrixXd cholesky_mat_;//TODO: Is this logical? (move out?)

    int nbrows_;
    int nbcols_;

    bool cholesky_old_mat_;

    Eigen::VectorXi row_indices_vec_;
    Eigen::VectorXi col_indices_vec_;
  };

}
#endif // MPC_WALKGEN_QP_MATRIX_H
