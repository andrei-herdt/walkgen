#pragma once
#ifndef MPC_WALKGEN_DYNAMICS_BUILDER_H
#define MPC_WALKGEN_DYNAMICS_BUILDER_H

#include <mpc-walkgen/types.h>

namespace MPCWalkgen{
class DynamicsBuilder{


	//
	// Public methods:
	//
public:
	DynamicsBuilder();

	~DynamicsBuilder();

	void Init(const MPCParameters *mpc_parameters);

	void Build(SystemOrder dynamics_order,
			LinearDynamics &dyn,
			double height,
			double sample_period_first,
			double sample_period_rest,
			int num_samples,
			bool actuation
	);

	//
	// Private methods:
	//
private:

	void BuildSecondOrder(
			LinearDynamics &dyn,
			double height,
			double sample_period_first,
			double sample_period_rest,
			int num_samples,
			bool actuation
	);
	void BuildThirdOrder(
			LinearDynamics &dyn,
			double height,
			double sample_period_first,
			double sample_period_rest,
			int num_samples
	);

	void BuildSecondOrderCoPOutput(
			LinearDynamicsMatrices &dyn,
			double height,
			double sample_period_first,
			double sample_period_rest,
			int num_samples,
			Derivative derivative
	);

	void BuildSecondOrderCoPInput(
			LinearDynamics &dyn,
			double height,
			double sample_period_first,
			double sample_period_rest,
			int num_samples
	);

	void BuildSecondOrderCoPInputGeneral(
			LinearDynamicsMatrices &dyn_mat,
			double height,
			double sample_period_first,
			double sample_period_rest,
			int num_samples
	);

	void BuildSecondOrderCoPInputDecoupled(
			LinearDynamicsMatrices &dyn_mat,
			const std::vector<CommonMatrixType> &d_state_mat_vec,
			const std::vector<CommonMatrixType> &d_input_mat_vec,
			const std::vector<CommonMatrixType> &d_state_mat_pow_vec,		// Recursively multiplied discrete state matrices
			const std::vector<CommonMatrixType> &rev_prod_dsmatrices_vec	// Recursively multiplied discrete state matrices
	);

	void BuildThirdOrder(LinearDynamicsMatrices &dyn,
			double height,
			double sample_period_first,
			double sample_period_rest,
			int num_samples,
			Derivative derivative
	);

	void ComputeDiscreteSSDynamics(
			LinearDynamics &dyn,
			const std::vector<double> &sampling_periods_vec
	);

	// Computes the discrete state matrix via \f[ e^{AT} \f]
	void ComputeDiscreteStateMatGeneral(
			Matrix2D &d_state_mat,
			double sample_period
	);

	void ComputeDiscreteInputMatGeneral(
			CommonMatrixType &d_input_mat,
			const CommonMatrixType &d_state_mat,
			const LinearDynamicsMatrices &dyn_mat
	);

	void ComputeDiscreteStateMat(
			LinearDynamics &dyn,
			double sample_period
	);

	// Computes the discrete input vector via \f[ A^{-1}(A_d-I)B \f]
	void ComputeDiscreteInputVec(
			LinearDynamics &dyn
	);

	// \brief (remark)We suppose that the state matrix is diagonal
	void ComputeDiscreteStateMatDecoupled(
			CommonMatrixType &d_state_mat,
			double sample_period
	);

	// \brief Build products of state matrices
	void BuildStateMatrixProducts(
			std::vector<CommonMatrixType> &d_state_mat_pow_vec,		// Forward order products
			std::vector<CommonMatrixType> &rev_prod_dsmatrices_vec,	// Backward order products
			const std::vector<CommonMatrixType> &d_state_mat_vec		// State matrices
	);

	//
	// Private data members:
	//
private:

	const MPCParameters *mpc_parameters_;

	// \name Intermediate elements for the computation of discrete state-space models
	// \{
	Vector2D eigenval_vec_;							//Vector of eigenvalues
	Matrix2D eigenvec_mat_, eigenvec_mat_inv_;		//Eigenvector matrices

	Matrix2D diag_exp_eig_mat_;						//\f[ e^{\lambda_1 T} \f]

	Matrix2D identity_mat_;
	Matrix2D precomp_input_mat_;  					// \f[ -A^{-1}\mathbb{I}B \f]

	Vector2D tmp_vec_;
	// \}

	Eigen::EigenSolver<CommonMatrixType> eigen_solver_;
};
}
#endif // MPC_WALKGEN_DYNAMICS_BUILDER_H
