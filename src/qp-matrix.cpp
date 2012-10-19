#include <mpc-walkgen/qp-matrix.h>
#include <mpc-walkgen/tools.h>

#include <Eigen/Cholesky>
#include <cmath>

using namespace MPCWalkgen;
using namespace Eigen;


QPMatrix::QPMatrix(const int num_rows_max, const int num_cols_max)
:constant_mat_(num_rows_max, num_cols_max)
,matrix_(num_rows_max, num_cols_max)
,cholesky_mat_(num_rows_max, num_cols_max)
,num_rows_max_(num_rows_max)
,num_cols_max_(num_cols_max)
,cholesky_old_mat_(true)
,row_indices_vec_(num_rows_max)
,col_indices_vec_(num_cols_max) {

	constant_mat_.setZero();
	matrix_.setZero();
	cholesky_mat_.setZero();

	for (int i = 0; i < num_rows_max_; ++i) {
		row_indices_vec_(i) = i;
	}
	for (int i = 0; i < num_cols_max_; ++i) {
		col_indices_vec_(i) = i;
	}
}

QPMatrix::~QPMatrix(){}

void QPMatrix::AddTerm(const MatrixRowMaj &mat,
		const int first_row, const int first_col) {
	// The following is optimized for row major matrices.
	// It has to be adapted if colum major ones are used.
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
		goal_mat_p = matrix_.data() + *row_p * num_cols_max_;
		col_p = first_col_p;
		for (int col = 0; col < mat.cols(); ++col) {
			// row major!
			*(goal_mat_p + *col_p) += *mat_p;
			++mat_p;
			++col_p;
		}
		++row_p;
	}

	cholesky_old_mat_ = true;

}

void QPMatrix::SetConstantPart(const CommonMatrixType &mat) {
	int num_rows = mat.rows();
	int num_cols = mat.cols();
	for (int i = 0; i <= num_rows; ++i) {
		for (int j = 0; j < num_cols; ++j) {
			//row major!
			*(constant_mat_.data() + col_indices_vec_(j) + row_indices_vec_(i) * num_cols_max_) = mat(i, j);
		}
	}
}

void QPMatrix::Reset() {
	matrix_.fill(0);
	cholesky_old_mat_ = true;
}

void QPMatrix::Resize(const int num_rows, const int num_cols) {
	num_rows_max_ = num_rows;
	num_cols_max_ = num_cols;
}

CommonMatrixType &QPMatrix::cholesky() {
	computeCholesky();
	return cholesky_mat_;
}

CommonMatrixType &QPMatrix::cholesky(CommonMatrixType &partialCholesky) {
	BuildCholesky(partialCholesky);
	return cholesky_mat_;
}

void QPMatrix::column_indices(const Eigen::VectorXi &order) {
	col_indices_vec_ = order;
}

void QPMatrix::row_indices(const Eigen::VectorXi &order) {
	row_indices_vec_ = order;
}

void QPMatrix::BuildCholesky(const CommonMatrixType &partialCholesky) {
	if (cholesky_old_mat_) {
		int imin = partialCholesky.rows();
		if (imin>0) {
			cholesky_mat_.block(0, 0, imin, imin) = partialCholesky;
			double tmp;
			for (int j=0; j<num_cols_max_; ++j){
				if (j>=imin){
					for (int i=0; i<j; ++i) {
						cholesky_mat_(j,i)=0;
					}
					tmp = matrix_(j,j);
					for (int k=0; k<j; ++k) {
						tmp -= pow2(cholesky_mat_(k,j));
					}
					if (tmp>kEps){
						cholesky_mat_(j,j) = sqrt(tmp);
					} else {
						cholesky_mat_(j,j) = 0;
					}

				}
				for (int i = std::max(j+1,imin); i<num_rows_max_; ++i) {
					tmp = matrix_(j,i);
					for (int k = 0; k < j; ++k) {
						tmp -= cholesky_mat_(k,j) * cholesky_mat_(k,i);
					}
					if (fabs(tmp) > kEps && fabs(cholesky_mat_(j,j)) > kEps) {
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
