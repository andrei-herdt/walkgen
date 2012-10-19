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

	void AddTerm(const MatrixRowMaj &mat,
			const int first_row,
			const int first_col
	);

	void SetConstantPart(const CommonMatrixType &mat
	);

	void Reset();

	void Resize(const int num_rows, const int num_cols);

	CommonMatrixType &cholesky();
	CommonMatrixType &cholesky(CommonMatrixType &partialCholesky);

	void column_indices(const Eigen::VectorXi &order);
	void row_indices(const Eigen::VectorXi &order);

	// \name Accessors and mutators
	// \{
	inline CommonMatrixType &operator()(void) {
		cholesky_old_mat_ = true;//TODO: hmm
		return matrix_;
	}
	inline const CommonMatrixType &operator()(void) const {
		return matrix_;
	}

	inline double &operator()(int row, int col = 0){return matrix_(row,col);}

	inline int num_rows_max() const	{return num_rows_max_;}
	inline int num_cols_max() const {return num_cols_max_;}
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

	int num_rows_max_;
	int num_cols_max_;

	bool cholesky_old_mat_;

	Eigen::VectorXi row_indices_vec_;//TODO: replace by std::vector
	Eigen::VectorXi col_indices_vec_;//TODO: replace by std::vector

};

}
#endif // MPC_WALKGEN_QP_MATRIX_H
