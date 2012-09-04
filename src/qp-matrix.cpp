#include <mpc-walkgen/qp-matrix.h>
#include <mpc-walkgen/tools.h>

#include <Eigen/Cholesky>
#include <cmath>

using namespace MPCWalkgen;
using namespace Eigen;


QPMatrix::QPMatrix(const int num_rows, const int num_cols)
:constant_mat_(num_rows, num_cols)
,matrix_(num_rows, num_cols)
,cholesky_mat_(num_rows, num_cols)
,num_rows_(num_rows)
,num_cols_(num_cols)
,cholesky_old_mat_(true)
,row_indices_vec_(num_rows)
,col_indices_vec_(num_cols) {

  constant_mat_.setZero();
  matrix_.setZero();
  cholesky_mat_.setZero();

  for (int i = 0; i < num_rows_; ++i) {
    row_indices_vec_(i) = i;
  }
  for (int i = 0; i < num_cols_; ++i) {
    col_indices_vec_(i) = i;
  }
}

QPMatrix::~QPMatrix(){}

void QPMatrix::addTerm(const EigenMatrixXdRM &mat,
                       const int first_row, const int first_col) {
                         // The following is optimized for row major matrices.
                         // It does not work for colum major ones.
                         // --------------------------------------------------
                         int newcol = 0;
                         int newrow = 0;
                         const double *mat_p = mat.data();
                         double *goal_mat_p = NULL;
                         int const *first_col_p = col_indices_vec_.data() + first_col;
                         int const *first_row_p = row_indices_vec_.data() + first_row;
                         const int *col_p = NULL;
                         const int *row_p = first_row_p;
                         
                         for (int row = 0; row < mat.rows(); ++row) {
                           goal_mat_p = matrix_.data() + *row_p * num_cols_;
                           col_p = first_col_p;
                           for (int col = 0; col < mat.cols(); ++col) {
                             // row major!
                             *(goal_mat_p + *col_p) += *mat_p;
                             ++mat_p;
                             ++col_p;
                           }
                           ++row_p;
                         }

                         // Equivalent, non-optimzed code:
                         // ------------------------------
                         /*   
                         for (int row = 0; row < mat.rows(); ++row) {
                         for (int col = 0; col < mat.cols(); ++col) {
                         matrix_(row_indices_vec_(first_row+row), col_indices_vec_(first_col + col)) += mat(row,col);
                         }
                         }
                         */				
                         cholesky_old_mat_ = true;

}

//TODO: Not used.
void QPMatrix::setTerm(const CommonMatrixType &mat, const int row, const int col) {
  int num_rows = mat.rows();
  int num_cols = mat.cols();
  for (int i = 0; i < num_rows; ++i) {
    for (int j = 0; j < num_cols; ++j) {
      // row major!
      *(matrix_.data() + col_indices_vec_(col + j) + row_indices_vec_(row + i) * num_cols_) = mat(i, j);
    }
  }
  cholesky_old_mat_ = true;
}

void QPMatrix::setConstantPart(const CommonMatrixType &mat) {
  int num_rows = mat.rows();
  int num_cols = mat.cols();
  for (int i = 0; i <= num_rows; ++i) {
    for (int j = 0; j < num_cols; ++j) {
      //row major!
      *(constant_mat_.data() + col_indices_vec_(j) + row_indices_vec_(i) * num_cols_) = mat(i, j);
    }
  }
}

void QPMatrix::reset() {
  matrix_.fill(0);
  cholesky_old_mat_ = true;
}

void QPMatrix::resize(const int num_rows, const int num_cols) {
  num_rows_ = num_rows;
  num_cols_ = num_cols;
  //matrix_.resize(nbRows_, nbCols_); //TODO: This should be avoided
}

CommonMatrixType &QPMatrix::cholesky() {
  computeCholesky();
  return cholesky_mat_;
}

CommonMatrixType &QPMatrix::cholesky(CommonMatrixType &partialCholesky) {
  computeCholesky(partialCholesky);
  return cholesky_mat_;
}

void QPMatrix::colOrder(const Eigen::VectorXi &order) {
  col_indices_vec_ = order;
}

void QPMatrix::rowOrder(const Eigen::VectorXi &order) {
  row_indices_vec_ = order;
}

void QPMatrix::computeCholesky(const CommonMatrixType &partialCholesky) {
  if (cholesky_old_mat_) {
    int imin = partialCholesky.rows();
    if (imin>0) {
      cholesky_mat_.block(0, 0, imin, imin) = partialCholesky;
      double tmp;
      for (int j=0; j<num_cols_; ++j){
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
        for (int i = std::max(j+1,imin); i<num_rows_; ++i) {
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
