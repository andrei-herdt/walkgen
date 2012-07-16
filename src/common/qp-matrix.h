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
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////

#include <vector>
#include <Eigen/Dense>
#include "types.h"

namespace MPCWalkgen{

  class QPMatrix{
  public:

    /// \name Typedefs
    /// \{
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> EigenMatrixXdRM;
    /// \}

    QPMatrix(const int nbrows = 0, const int nbcols = 0);

    ~QPMatrix();

    void addTerm(const Eigen::MatrixXd & mat,
      const int row = 0, const int col = 0);

    void setTerm(const Eigen::MatrixXd & mat,
      const int row = 0, const int col = 0);

    void setConstantPart(const Eigen::MatrixXd & mat);

    void reset();

    void resize(const int nbrows, const int nbcols);

    Eigen::MatrixXd & cholesky();
    Eigen::MatrixXd & cholesky(Eigen::MatrixXd & partialCholesky);

    void colOrder(const Eigen::VectorXi & order);
    void rowOrder(const Eigen::VectorXi & order);


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

/*! \fn MPCWalkgen::QPMatrix::QPMatrix(const int nbrows, const int nbcols,
const int nbrowsMax=1, const int nbcolsMax=1)
* \brief Constructor. nbrowsMax/nbcolsMax must be greater or equal than nbrows/nbcols
* \param nbrows Declared row number of dense matrix
* \param nbcols Declared col number of dense matrix
* \param nbrowsMax number of rows of storage matrix
* \param nbcolsMax number of cols of storage matrix
*/

/*! \fn void MPCWalkgen::QPMatrix::addTerm(const Eigen::MatrixXd & mat,
const int row = 0, const int col = 0)
* \brief Add the content of matrix mat into the QPMatrix, starting at position (row/col)
*/

/*! \fn void MPCWalkgen::QPMatrix::setConstantPart(const Eigen::MatrixXd & mat)
* \brief Define the constant part of the QPMatrix
*/

/*! \fn void MPCWalkgen::QPMatrix::reset(const bool withConstantPart = false)
* \brief Erase the QPMatrix
* \param withConstantPart If true, Set QPMatrix to it constant part
*/

/*! \fn void MPCWalkgen::QPMatrix::resize(const int nbrows, const int nbcols=1,
const bool preserve=false, const bool withConstantPart = false)
* \brief define the new declared dimension of dense matrix
* \param preserve if true, the content will be conserved. Else, reset method will be called
* \param withConstantPart for reset method, only if preserve=true
*/

/*! \fn Eigen::MatrixXd & MPCWalkgen::QPMatrix::operator()(void)
* \brief Return the storage matrix
*/

/*! \fn double & MPCWalkgen::QPMatrix::operator()(int row, int col=0)
* \brief return the value of the matrix at (row,col)
*/

/*! \fn Eigen::MatrixXd & MPCWalkgen::QPMatrix::dense()
* \brief return the dense matrix
*/

/*! \fn int MPCWalkgen::QPMatrix::nbrows() const
* \brief return the dense matrix number of rows
*/

/*! \fn int MPCWalkgen::QPMatrix::nbcols() const
* \brief return the dense matrix number of cols
*/

/*! \fn int MPCWalkgen::QPMatrix::nbrowsMax() const
* \brief return the storage matrix number of rows
*/

/*! \fn int MPCWalkgen::QPMatrix::nbcolsMax() const
* \brief return the storage matrix number of cols
*/

#endif // MPC_WALKGEN_QP_MATRIX_H
