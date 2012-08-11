#include "qp-matrix.h"
#include "tools.h"
#include <Eigen/Cholesky>
#include <cmath>

using namespace MPCWalkgen;
using namespace Eigen;


QPMatrix::QPMatrix(const int nbrows, const int nbcols)
:constant_mat_(nbrows, nbcols)
,matrix_(nbrows, nbcols)
,cholesky_mat_(nbrows, nbcols)
,nbrows_(nbrows)
,nbcols_(nbcols)
,cholesky_old_mat_(true)
,row_indices_vec_(nbrows)
,col_indices_vec_(nbcols) {

  constant_mat_.setZero();
  matrix_.setZero();
  cholesky_mat_.setZero();

  for (int i = 0; i < nbrows_; ++i) {
    row_indices_vec_(i) = i;
  }
  for (int i = 0; i < nbcols_; ++i) {
    col_indices_vec_(i) = i;
  }
}

QPMatrix::~QPMatrix(){}

void QPMatrix::addTerm(const MatrixXd &mat,
                       const int row, const int col) {
                         int nbrows = mat.rows();
                         int nbcols = mat.cols();
                         int newcol = 0;
                         int newrow = 0;
                         for (int i = 0; i < nbrows; ++i) {
                           newrow = row_indices_vec_(row + i);
                           for (int j = 0; j < nbcols; ++j) {
                             newcol = col_indices_vec_(col + j);
                             // row major!
                             *(matrix_.data() + newcol + newrow * nbcols_) += mat(i, j);
                           }
                         }
                         cholesky_old_mat_ = true;
}

void QPMatrix::setTerm(const MatrixXd &mat,
                       const int row, const int col) {
                         int nbrows = mat.rows();
                         int nbcols = mat.cols();
                         for (int i = 0; i < nbrows; ++i) {
                           for (int j = 0; j < nbcols; ++j) {
                             // row major!
                             *(matrix_.data() + col_indices_vec_(col + j) + row_indices_vec_(row + i) * nbcols_) = mat(i, j);
                           }
                         }
                         cholesky_old_mat_ = true;
}

void QPMatrix::setConstantPart(const MatrixXd &mat) {
  int nbrows = mat.rows();
  int nbcols = mat.cols();
  for (int i = 0; i <= nbrows; ++i) {
    for (int j = 0; j < nbcols; ++j) {
      //row major!
      *(constant_mat_.data() + col_indices_vec_(j) + row_indices_vec_(i) * nbcols_) = mat(i, j);
    }
  }
}

void QPMatrix::reset() {
  matrix_.fill(0);
  cholesky_old_mat_ = true;
}

void QPMatrix::resize(const int nbrows, const int nbcols) {
  nbrows_ = nbrows;
  nbcols_ = nbcols;
  //matrix_.resize(nbRows_, nbCols_); //TODO: This should be avoided
}

MatrixXd &QPMatrix::cholesky() {
  computeCholesky();
  return cholesky_mat_;
}

MatrixXd &QPMatrix::cholesky(MatrixXd &partialCholesky) {
  computeCholesky(partialCholesky);
  return cholesky_mat_;
}

void QPMatrix::colOrder(const Eigen::VectorXi &order) {
  col_indices_vec_ = order;
}

void QPMatrix::rowOrder(const Eigen::VectorXi &order) {
  row_indices_vec_ = order;
}

void QPMatrix::computeCholesky(const MatrixXd &partialCholesky) {
  if (cholesky_old_mat_) {
    int imin = partialCholesky.rows();
    if (imin>0) {
      cholesky_mat_.block(0, 0, imin, imin) = partialCholesky;
      double tmp;
      for (int j=0; j<nbcols_; ++j){
        if (j>=imin){
          for (int i=0; i<j; ++i) {
            cholesky_mat_(j,i)=0;
          }
          tmp = matrix_(j,j);
          for (int k=0; k<j; ++k) {
            tmp -= pow2(cholesky_mat_(k,j));
          }
          if (tmp>EPSILON){
            cholesky_mat_(j,j) = sqrt(tmp);
          } else {
            cholesky_mat_(j,j) = 0;
          }

        }
        for (int i = std::max(j+1,imin); i<nbrows_; ++i) {
          tmp = matrix_(j,i);
          for (int k = 0; k < j; ++k) {
            tmp -= cholesky_mat_(k,j) * cholesky_mat_(k,i);
          }
          if (fabs(tmp) > EPSILON && fabs(cholesky_mat_(j,j)) > EPSILON){
            cholesky_mat_(j,i) = tmp / cholesky_mat_(j,j);
          } else {
            cholesky_mat_(j,i) = 0;
          }
        }
      }

    } else {
      cholesky_mat_ = matrix_.llt().matrixL().transpose();
    }
    cholesky_old_mat_ = false;
  }
}

void QPMatrix::computeCholesky() {
  if (cholesky_old_mat_) {
    cholesky_mat_ = matrix_.llt().matrixL().transpose();
    cholesky_old_mat_ = false;
  }
}
