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

    void setTerm(const Eigen::VectorXd &vec, const int row);

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

    Eigen::VectorXi rowOrder_;
  };
}

/*! \fn MPCWalkgen::QPVector::QPVector(const int nbRows, const int nbCols,
const int nbRowsMax=1, const int nbColsMax=1)
* \brief Constructor. nbRowsMax/nbColsMax must be greater or equal than nbRows/nbCols
* \param nbRows Declared row number of dense matrix
* \param nbCols Declared col number of dense matrix
* \param nbRowsMax number of rows of storage matrix
* \param nbColsMax number of cols of storage matrix
*/

/*! \fn void MPCWalkgen::QPVector::addTerm(const Eigen::MatrixXd &mat,
const int row = 0, const int col = 0)
* \brief Add the content of matrix mat into the QPVector, starting at position (row/col)
*/

/*! \fn void MPCWalkgen::QPVector::setConstantPart(const Eigen::MatrixXd &mat)
* \brief Define the constant part of the QPVector
*/

/*! \fn void MPCWalkgen::QPVector::reset(const bool withConstantPart = false)
* \brief Erase the QPVector
* \param withConstantPart If true, Set QPVector to it constant part
*/

/*! \fn void MPCWalkgen::QPVector::resize(const int nbRows, const int nbCols=1,
const bool preserve=false, const bool withConstantPart = false)
* \brief define the new declared dimension of dense matrix
* \param preserve if true, the content will be conserved. Else, reset method will be called
* \param withConstantPart for reset method, only if preserve=true
*/

/*! \fn Eigen::MatrixXd &MPCWalkgen::QPVector::operator()(void)
* \brief Return the storage matrix
*/

/*! \fn double &MPCWalkgen::QPVector::operator()(int row, int col=0)
* \brief return the value of the matrix at (row,col)
*/

/*! \fn Eigen::MatrixXd &MPCWalkgen::QPVector::dense()
* \brief return the dense matrix
*/

/*! \fn int MPCWalkgen::QPVector::nbRows() const
* \brief return the dense matrix number of rows
*/

/*! \fn int MPCWalkgen::QPVector::nbCols() const
* \brief return the dense matrix number of cols
*/

/*! \fn int MPCWalkgen::QPVector::nbRowsMax() const
* \brief return the storage matrix number of rows
*/

/*! \fn int MPCWalkgen::QPVector::nbColsMax() const
* \brief return the storage matrix number of cols
*/

#endif // MPC_WALKGEN_QP_VECTOR_H
