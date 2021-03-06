#include <mpc-walkgen/qp-builder.h>
#include <mpc-walkgen/qp-matrix.h>
#include <mpc-walkgen/qp-vector.h>
#include <mpc-walkgen/tools.h>
#include <mpc-walkgen/debug.h>

#include <iostream>
#include <algorithm>

using namespace MPCWalkgen;
using namespace std;

QPBuilder::QPBuilder(HeuristicPreview *preview, QPSolver *solver,
		Reference *pos_ref, Reference *vel_ref, Reference *cp_ref, RigidBodySystem *robot, MPCParameters *mpc_parameters,
		RealClock *clock, double *last_des_cop_x, double *last_des_cop_y,
		double *last_first_contr_x, double *last_first_contr_y)
:preview_(preview)
,solver_(solver)
,robot_(robot)
,pos_ref_(pos_ref)
,vel_ref_(vel_ref)
,cp_ref_(cp_ref)
,mpc_parameters_(mpc_parameters)
,tmp_vec_(1)
,tmp_vec2_(1)
,tmp_vec3_(1)
,tmp_vec4_(1)
,tmp_mat_(1,1)
,tmp_mat2_(1,1)
,current_time_(0.)
,clock_(clock)
,last_des_cop_x_(last_des_cop_x)
,last_des_cop_y_(last_des_cop_y)
,last_first_contr_x_(last_first_contr_x)
,last_first_contr_y_(last_first_contr_y)
{}

QPBuilder::~QPBuilder() {}


void QPBuilder::PrecomputeObjective() {
	//TODO: The following has already been done in Walkgen::Init()
	// Define order of variables:
	// --------------------------
	int num_samples_max = 0;
	if (mpc_parameters_->mapping == ZERO_MAP) {
		num_samples_max = mpc_parameters_->num_samples_horizon_max;
	} else if (mpc_parameters_->mapping == CONST_MAP) {
		num_samples_max = mpc_parameters_->num_samples_state;
	}
	assert(num_samples_max > 0);

	int num_unst_modes = 0;
	if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		num_unst_modes = 1;
	}

	//TODO: Why setting order here again?
	/*
	Eigen::VectorXi order(2*(num_samples + num_unst_modes));
	for (int i = 0; i < num_samples + num_unst_modes; ++i) {
		//order(i) = 2 * i;
		//order(i + num_samples + num_unst_modes) = 2 * i + 1;
		order(i) = i;
		order(num_samples + num_unst_modes + i) = num_samples + num_unst_modes + i;
	}
	chol.row_indices(order);
	chol.column_indices(order);
	 */

	// Set Theta:
	// ----------
	QPMatrix chol(2*(num_samples_max + num_unst_modes), 2 * (num_samples_max + num_unst_modes));

	int num_modes = mpc_parameters_->penalties.contr_moves.size();
	int num_dynamics = static_cast<int>(robot_->com()->dynamics_qp_vec().size());
	curr_cop_variant_.resize(num_dynamics * num_modes);
	const_hessian_mat_.resize(num_dynamics * num_modes);
	const_hessian_n_mat_.resize(num_dynamics * num_modes);
	const_cholesky_.resize(num_dynamics * num_modes);
	state_variant_.resize(num_dynamics * num_modes);
	select_variant_.resize(num_dynamics * num_modes);
	ref_variant_vel_.resize(num_dynamics * num_modes);
	ref_variant_pos_.resize(num_dynamics * num_modes);
	ref_variant_cp_.resize(num_dynamics * num_modes);
	cop_pen_mat_vec_.resize(num_dynamics * num_modes);
	dcop_pen_mat_vec_.resize(num_dynamics * num_modes);
	dcop_mat_vec_.resize(num_dynamics * num_modes);
	dcop_mat_tr_vec_.resize(num_dynamics * num_modes);

	// CP centering:
	// -------------
	u_trans_v_mat_vec_.resize(num_dynamics * num_modes);
	v_trans_v_mat_vec_.resize(num_dynamics * num_modes);
	v_trans_vc_mat_vec_.resize(num_dynamics * num_modes);
	v_trans_s_mat_vec_.resize(num_dynamics * num_modes);
	u_trans_vc_mat_vec_.resize(num_dynamics * num_modes);
	u_trans_s_mat_vec_.resize(num_dynamics * num_modes);
	u_trans_ref_mat_vec_.resize(num_dynamics * num_modes);
	v_trans_ref_mat_vec_.resize(num_dynamics * num_modes);

	cp_fp_pen_mat_vec_.resize(num_dynamics * num_modes);
	vel_pen_mat_vec_.resize(num_dynamics * num_modes);

	int ns_st = 0;
	if (mpc_parameters_->mapping == CONST_MAP) {
		ns_st = mpc_parameters_->num_samples_state;
	} else if (mpc_parameters_->mapping == ZERO_MAP) {
		ns_st = mpc_parameters_->num_samples_horizon_max;
	}

	for (int mode_num = 0; mode_num < num_modes; ++mode_num) {
		CommonMatrixType cp_fp_pen_fixed_mat = CommonMatrixType::Identity(ns_st + num_unst_modes, ns_st + num_unst_modes) * mpc_parameters_->penalties.cp_fp[mode_num];
		cp_fp_pen_mat_vec_[mode_num] = cp_fp_pen_fixed_mat;
		CommonMatrixType vel_pen_fixed_mat = CommonMatrixType::Identity(ns_st + num_unst_modes, ns_st + num_unst_modes) * mpc_parameters_->penalties.vel[mode_num];
		vel_pen_mat_vec_[mode_num] = vel_pen_fixed_mat;

		if (mpc_parameters_->mapping == ZERO_MAP) {
			double first_period = mpc_parameters_->period_qpsample;
			for (int s = 7; s > 3; s--) {
				for (int i = 0; i < 4; i++) {
					int samples_left = mpc_parameters_->GetMPCSamplesLeft(first_period);
					int mat_num = samples_left + mode_num * num_dynamics;

					cp_fp_pen_fixed_mat(s, s) = mpc_parameters_->penalties.cp_fp[mode_num];// * (1. - samples_left / 19.);
					cp_fp_pen_mat_vec_[mat_num] = cp_fp_pen_fixed_mat;
					vel_pen_fixed_mat(s, s) = mpc_parameters_->penalties.vel[mode_num];// * (1. - samples_left / 19.);
					vel_pen_mat_vec_[mat_num] = vel_pen_fixed_mat;

					first_period -= mpc_parameters_->period_recomputation;
				}
				//cp_fp_pen_fixed_mat(s, s) = 0.;
				//vel_pen_fixed_mat(s, s) = 0.;
			}
			for (int s = 3; s >= 0; s--) {
				int samples_left = mpc_parameters_->GetMPCSamplesLeft(first_period);
				int mat_num = samples_left + mode_num * num_dynamics;

				cp_fp_pen_fixed_mat(s, s) = mpc_parameters_->penalties.cp_fp[mode_num];// * (1. - samples_left / 19.);
				cp_fp_pen_mat_vec_[mat_num] = cp_fp_pen_fixed_mat;
				//cp_fp_pen_fixed_mat(s, s) = 0.;
				vel_pen_fixed_mat(s, s) = mpc_parameters_->penalties.vel[mode_num];// * (1. - samples_left / 19.);
				vel_pen_mat_vec_[mat_num] = vel_pen_fixed_mat;
				//vel_pen_fixed_mat(s, s) = 0.;

				first_period -= mpc_parameters_->period_recomputation;
			}
		}
	}

	// Precompute:
	// -----------
	for (int mode_num = 0; mode_num < num_modes; ++mode_num) {
		double first_period = mpc_parameters_->period_qpsample;
		for (int dyn_num = 0; dyn_num < num_dynamics; dyn_num++) {
			int samples_left = mpc_parameters_->GetMPCSamplesLeft(first_period);
			if (mpc_parameters_->mapping == CONST_MAP) {
				samples_left = 0;
			}
			int mat_num = samples_left + mode_num * num_dynamics;

			const LinearDynamics &dyn 				= robot_->com()->dynamics_qp_vec()[samples_left];
			const LinearDynamicsMatrices &pos_dyn = robot_->com()->dynamics_qp_vec()[samples_left].pos;
			const LinearDynamicsMatrices &vel_dyn = robot_->com()->dynamics_qp_vec()[samples_left].vel;
			const LinearDynamicsMatrices &cop_dyn = robot_->com()->dynamics_qp_vec()[samples_left].cop;
			const LinearDynamicsMatrices &cp_dyn = robot_->com()->dynamics_qp_vec()[samples_left].cp;

			int ns_c = dyn.input_map_mat.cols();
			if (mpc_parameters_->formulation == DECOUPLED_MODES) {
				ns_c--;
			}
			if (mpc_parameters_->mapping == ZERO_MAP) {
				ns_st = ns_c;
			}

			if (mpc_parameters_->formulation == DECOUPLED_MODES) {
				cp_fp_pen_mat_vec_[mat_num](ns_st, ns_st) = 0.;	// CP variable
			}

			CommonMatrixType hessian_mat(ns_c + num_unst_modes, ns_c + num_unst_modes);

			dcop_mat_vec_[mat_num] = CommonMatrixType::Identity(ns_st + num_unst_modes, ns_st + num_unst_modes);
			if (mpc_parameters_->formulation == DECOUPLED_MODES) {
				dcop_mat_vec_[mat_num](ns_st, ns_st) = 0.;
			}
			for (int row = 1; row < ns_st; row++) {
				dcop_mat_vec_[mat_num](row, row - 1) = -1.;
			}
			dcop_mat_tr_vec_[mat_num] = dcop_mat_vec_[mat_num].transpose();

			dcop_pen_mat_vec_[mat_num] = CommonMatrixType::Identity(ns_st + num_unst_modes, ns_st + num_unst_modes)
			* mpc_parameters_->penalties.contr_moves[mode_num];
			int i = 0;
			for (; i < mpc_parameters_->num_samples_first_fine_period; i++) {
				dcop_pen_mat_vec_[mat_num](i, i) = mpc_parameters_->penalties.first_contr_moves;
			}
			for (; i < mpc_parameters_->num_samples_first_fine_period + mpc_parameters_->num_samples_first_coarse_period - 1; i++) {
				dcop_pen_mat_vec_[mat_num](i, i) = mpc_parameters_->penalties.second_contr_moves;
			}


			// Q = beta*Uz^(-T)*Uv^T*Uv*Uz^(-1)
			//tmp_mat_.noalias() = cop_dyn.input_mat_inv_tr * vel_dyn.input_mat_tr * vel_dyn.input_mat * cop_dyn.input_mat_inv;
			tmp_mat_.noalias() = vel_dyn.input_mat_tr * vel_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st) * vel_dyn.input_mat;
			hessian_mat = tmp_mat_;
			// Q += alpha*Uz^(-T)*Uz^(-1)
			//tmp_mat_.noalias() = cop_dyn.input_mat_inv_tr * contr_weighting_mat * cop_dyn.input_mat_inv;
			hessian_mat += cop_dyn.input_mat_tr * dcop_mat_tr_vec_[mat_num].block(0, 0, ns_st, ns_st) *
					dcop_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st) * dcop_mat_vec_[mat_num].block(0, 0, ns_st, ns_st) * cop_dyn.input_mat;
			// Q += delta*Uz^(-T)*Up^T*Up*Uz^(-1)
			//tmp_mat_.noalias() = cop_dyn.input_mat_inv_tr * pos_dyn.input_mat_tr * pos_dyn.input_mat * cop_dyn.input_mat_inv;
			tmp_mat_.noalias() = pos_dyn.input_mat_tr * pos_dyn.input_mat;
			hessian_mat +=  mpc_parameters_->penalties.pos[mode_num] * tmp_mat_;
			// Q += epsilon*Uz^(-T)*Uxi^T*Uxi*Uz^(-1)
			//tmp_mat_.noalias() = cop_dyn.input_mat_inv_tr * cp_dyn.input_mat_tr * cp_dyn.input_mat * cop_dyn.input_mat_inv;

			CommonMatrixType cp_pen_mat = CommonMatrixType::Identity(ns_st + num_unst_modes, ns_st + num_unst_modes) * mpc_parameters_->penalties.cp[mode_num];
			if (mpc_parameters_->formulation == DECOUPLED_MODES) {
				cp_pen_mat(ns_st, ns_st) = 0.;	// CP variable
			}
			//cp_pen_mat(0, 0) = mpc_parameters_->penalties.cp[mode_num] * first_period / mpc_parameters_->period_qpsample;
			tmp_mat_.noalias() = cp_dyn.input_mat_tr * cp_pen_mat.block(0, 0, ns_st, ns_st) * cp_dyn.input_mat;
			hessian_mat +=  tmp_mat_;
			// CP centering:
			// -------------
			//cp_fp_pen_mat_vec_[mat_num](num_samples - 1 , num_samples - 1) *= 1000;
			//cp_fp_pen_mat_vec_[mat_num](0, 0) = mpc_parameters_->penalties.cp_fp[mode_num] * first_period / mpc_parameters_->period_qpsample;

			tmp_mat_.noalias() = cp_dyn.input_mat_tr * cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st) * cp_dyn.input_mat;
			hessian_mat +=  tmp_mat_;
			// CoP centering:
			// -------------
			// Q += gamma*I
			cop_pen_mat_vec_[mat_num] = CommonMatrixType::Identity(ns_st + num_unst_modes, ns_st + num_unst_modes) * mpc_parameters_->penalties.cop[mode_num];
			if (mpc_parameters_->formulation == DECOUPLED_MODES) {
				cop_pen_mat_vec_[mat_num](ns_st, ns_st) = 0.;	// CP variable
			}
			// TODO: Hard coded for 5 ms rate
			for (int i = 0; i < mpc_parameters_->num_samples_first_fine_period; i++) {
				cop_pen_mat_vec_[mat_num](i, i) = /*0. / 100. **/ mpc_parameters_->penalties.cop[mode_num];
			}
			for (int i = mpc_parameters_->num_samples_first_fine_period; i < mpc_parameters_->num_samples_first_fine_period + mpc_parameters_->num_samples_first_coarse_period - 1; i++) {
				cop_pen_mat_vec_[mat_num](i, i) = /*20. / 100.  **/ mpc_parameters_->penalties.cop[mode_num];
			}
			hessian_mat += cop_pen_mat_vec_[mat_num];
			//contr_val_pen_mat_vec_[mat_num](0, 0) = mpc_parameters_->penalties.cop[mode_num] * first_period / mpc_parameters_->period_qpsample;

			// Assign hessian:
			// ---------------
			const_hessian_n_mat_[mat_num] = hessian_mat;

			// Map input:
			// ----------
			const CommonMatrixType &input_map_mat = robot_->com()->dynamics_qp_vec()[samples_left].input_map_mat;
			const CommonMatrixType &input_map_mat_tr = robot_->com()->dynamics_qp_vec()[samples_left].input_map_mat_tr;
			const_hessian_n_mat_[mat_num] = input_map_mat_tr * const_hessian_n_mat_[mat_num] * input_map_mat;

			/*
			chol.Reset(0.);
			chol.AddTerm(const_hessian_n_mat_[mat_num], 0, 0);
			chol.AddTerm(const_hessian_n_mat_[mat_num], const_hessian_n_mat_[mat_num].rows(), const_hessian_n_mat_[mat_num].cols());

			const_cholesky_[mat_num] = chol.cholesky();
			 */

			// beta*Uz^(-T)*Uv*(Sv - Uv*Uz^(-1)*Sz)
			//tmp_mat_.noalias() = vel_dyn.state_mat - vel_dyn.input_mat * cop_dyn.input_mat_inv * cop_dyn.state_mat;
			//state_variant_[mat_num] = cop_dyn.input_mat_inv_tr * vel_dyn.input_mat_tr * mpc_parameters->weights.vel[mode_num] * tmp_mat_;
			tmp_mat_.noalias() = vel_dyn.state_mat;//New - vel_dyn.input_mat * cop_dyn.state_mat;
			state_variant_[mat_num] = vel_dyn.input_mat_tr * vel_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st) * tmp_mat_;
			// delta*Uz^(-T)*Up*(Sp - Up*Uz^(-1)*Sz)
			//tmp_mat_.noalias() = pos_dyn.state_mat - pos_dyn.input_mat * cop_dyn.input_mat_inv * cop_dyn.state_mat;
			//state_variant_[mat_num] += cop_dyn.input_mat_inv_tr * pos_dyn.input_mat_tr * mpc_parameters_->weights.pos[mode_num] * tmp_mat_;
			tmp_mat_.noalias() = pos_dyn.state_mat;//New - pos_dyn.input_mat * cop_dyn.state_mat;
			state_variant_[mat_num] += pos_dyn.input_mat_tr * mpc_parameters_->penalties.pos[mode_num] * tmp_mat_;
			// epsilon*Uz^(-T)*Uxi*(Sp - Uxi*Uz^(-1)*Sz)
			//tmp_mat_.noalias() = cp_dyn.state_mat - cp_dyn.input_mat * cop_dyn.input_mat_inv * cop_dyn.state_mat;
			//state_variant_[mat_num] += cop_dyn.input_mat_inv_tr * cp_dyn.input_mat_tr * mpc_parameters_->weights.cp[mode_num] * tmp_mat_;
			tmp_mat_.noalias() = cp_dyn.state_mat;//New - cp_dyn.input_mat * cop_dyn.state_mat;
			state_variant_[mat_num] += cp_dyn.input_mat_tr * cp_pen_mat.block(0, 0, ns_st, ns_st) * tmp_mat_;

			// - alpha*Uz^(-T)*Uz^(-1)*Sz
			//state_variant_[mat_num] -= cop_dyn.input_mat_inv_tr * mpc_parameters_->weights.control[mode_num] * cop_dyn.input_mat_inv * cop_dyn.state_mat;
			//new: state_variant_[mat_num] -= mpc_parameters_->weights.control[mode_num] * cop_dyn.state_mat;

			// alpha*Uz^(-T)*Uz^(-1)
			//select_variant_[mat_num]  = cop_dyn.input_mat_inv_tr * mpc_parameters_->weights.control[mode_num] * cop_dyn.input_mat_inv;
			//select_variant_[mat_num]  = mpc_parameters_->weights.control[mode_num] * identity_mat.block(0, 0, num_samples + num_unst_modes, num_samples);
			// beta*Uz^(-T)*Uv^T*Uv*Uz^(-1)
			//select_variant_[mat_num] += cop_dyn.input_mat_inv_tr * vel_dyn.input_mat_tr * mpc_parameters_->weights.vel[mode_num] * vel_dyn.input_mat * cop_dyn.input_mat_inv;
			select_variant_[mat_num] = vel_dyn.input_mat_tr * vel_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st) *  vel_dyn.input_mat.block(0, 0, ns_st, ns_st);
			// delta*Uz^(-T)*Up^T*Up*Uz^(-1)
			//select_variant_[mat_num] += cop_dyn.input_mat_inv_tr * pos_dyn.input_mat_tr * mpc_parameters_->weights.pos[mode_num] * pos_dyn.input_mat * cop_dyn.input_mat_inv;
			select_variant_[mat_num] += pos_dyn.input_mat_tr * mpc_parameters_->penalties.pos[mode_num] * pos_dyn.input_mat.block(0, 0, ns_st, ns_st);
			// epsilon*Uz^(-T)*Uxi^T*Uxi*Uz^(-1)
			//select_variant_[mat_num] += cop_dyn.input_mat_inv_tr * cp_dyn.input_mat_tr * mpc_parameters_->weights.cp[mode_num] * cp_dyn.input_mat * cop_dyn.input_mat_inv;
			select_variant_[mat_num] += cp_dyn.input_mat_tr * cp_pen_mat.block(0, 0, ns_st, ns_st) * cp_dyn.input_mat.block(0, 0, ns_st, ns_st);
			// + r*theta^(T)*theta^T
			//select_variant_[mat_num] += dcop_mat_tr_vec_[mat_num].block(0, 0, ns_st + num_unst_modes, ns_st) * dcop_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st) * dcop_mat_vec_[mat_num].block(0, 0, ns_st, ns_st);

			// - beta*Uz^(-T)*Uv^T
			//ref_variant_vel_[mat_num] = -cop_dyn.input_mat_inv_tr * vel_dyn.input_mat_tr * mpc_parameters_->weights.vel[mode_num];
			ref_variant_vel_[mat_num] = -vel_dyn.input_mat_tr * vel_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st);
			// - delta*Uz^(-T)*Up^T
			//ref_variant_pos_[mat_num] = -cop_dyn.input_mat_inv_tr * pos_dyn.input_mat_tr * mpc_parameters_->weights.pos[mode_num];
			ref_variant_pos_[mat_num] = -pos_dyn.input_mat_tr * mpc_parameters_->penalties.pos[mode_num];
			// - epsilon*Uz^(-T)*Uxi^T
			//ref_variant_cp_[mat_num] = -cop_dyn.input_mat_inv_tr * cp_dyn.input_mat_tr * mpc_parameters_->weights.cp[mode_num];
			ref_variant_cp_[mat_num] = - cp_dyn.input_mat_tr * cp_pen_mat.block(0, 0, ns_st, ns_st);

			CommonVectorType e_vec = CommonVectorType::Zero(ns_st);
			e_vec(0) = 1.;
			curr_cop_variant_[mat_num] = - dcop_mat_tr_vec_[mat_num].block(0, 0, ns_st + num_unst_modes, ns_st) * dcop_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st) * e_vec;


			// CP centering:
			// -------------
			u_trans_v_mat_vec_[mat_num] = cp_dyn.input_mat_tr * cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st) * cp_dyn.input_mat.block(0, 0, ns_st, ns_st);
			u_trans_v_mat_vec_[mat_num] -= cp_dyn.input_mat_tr * cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st);
			tmp_mat2_.noalias() = u_trans_v_mat_vec_[mat_num] + select_variant_[mat_num];
			u_trans_v_mat_vec_[mat_num] = input_map_mat_tr * tmp_mat2_;

			v_trans_v_mat_vec_[mat_num] = cp_dyn.input_mat_tr.block(0, 0, ns_st, ns_st) * cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st) * cp_dyn.input_mat.block(0, 0, ns_st, ns_st);
			v_trans_v_mat_vec_[mat_num] -= cp_dyn.input_mat_tr.block(0, 0, ns_st, ns_st) * cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st);
			v_trans_v_mat_vec_[mat_num] -= cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st) * cp_dyn.input_mat.block(0, 0, ns_st, ns_st);
			v_trans_v_mat_vec_[mat_num] += cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st);
			v_trans_v_mat_vec_[mat_num] += select_variant_[mat_num].block(0, 0, ns_st, ns_st);

			v_trans_vc_mat_vec_[mat_num] = cp_dyn.input_mat_tr.block(0, 0, ns_st, ns_st) * cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st) * cp_dyn.input_mat.block(0, 0, ns_st, ns_st);
			v_trans_vc_mat_vec_[mat_num] -= cp_dyn.input_mat_tr.block(0, 0, ns_st, ns_st) * cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st);
			v_trans_vc_mat_vec_[mat_num] -= cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st) * cp_dyn.input_mat.block(0, 0, ns_st, ns_st);
			v_trans_vc_mat_vec_[mat_num] += cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st);

			tmp_mat_.noalias() = cp_dyn.state_mat;//New - cp_dyn.input_mat * cop_dyn.state_mat;
			v_trans_s_mat_vec_[mat_num] = cp_dyn.input_mat_tr.block(0, 0, ns_st, ns_st) * cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st) * tmp_mat_;
			v_trans_s_mat_vec_[mat_num] -= cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st) * tmp_mat_;

			tmp_mat2_.noalias() = cp_dyn.input_mat_tr * cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st) * tmp_mat_;
			u_trans_s_mat_vec_[mat_num] = input_map_mat_tr * tmp_mat2_;

			tmp_mat2_.noalias() = cp_dyn.input_mat_tr * cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st) * cp_dyn.input_mat.block(0, 0, ns_st, ns_st);
			u_trans_vc_mat_vec_[mat_num] = input_map_mat_tr * tmp_mat2_;
			tmp_mat2_.noalias() = cp_dyn.input_mat_tr * cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st);
			u_trans_vc_mat_vec_[mat_num] -= input_map_mat_tr * tmp_mat2_;

			tmp_mat2_.noalias() = - cp_dyn.input_mat_tr * cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st);
			u_trans_ref_mat_vec_[mat_num] = input_map_mat_tr * tmp_mat2_;

			v_trans_ref_mat_vec_[mat_num] = - cp_dyn.input_mat_tr.block(0, 0, ns_st, ns_st) * cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st);
			v_trans_ref_mat_vec_[mat_num] += cp_fp_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st);

			first_period -= mpc_parameters_->period_recomputation;
		}
	}

	state_trans_mat_ = Matrix2D::Zero();
	state_trans_mat_(0, 0) = 1.;
	state_trans_mat_(0, 1) = -1. / sqrt(kGravity / robot_->com()->state().z[0]);
	state_trans_mat_(1, 0) = 1.;
	state_trans_mat_(1, 1) = 1. / sqrt(kGravity / robot_->com()->state().z[0]);

}

void QPBuilder::BuildProblem(MPCSolution &solution) {
	assert(mpc_parameters_->num_samples_contr > 0);

	// DIMENSION OF QP:
	// ----------------
	int num_variables = 2 * mpc_parameters_->num_samples_contr +			// com
			2 * solution.support_states_vec.back().step_number;				// Foot placement

	if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		num_variables += 2;
	}

	solver_->uv_bounds_vec().Reset(kInf);
	solver_->lv_bounds_vec().Reset(-kInf);
	solver_->num_var(num_variables);

	//int obj_counter = clock_->StartCounter();
	BuildObjective(solution);
	//clock_->StopCounter(obj_counter);

	BuildEqualityConstraints(solution);

	if (mpc_parameters_->is_ineq_constr) {
		BuildInequalityConstraints(solution);
	}

	if (mpc_parameters_->is_terminal_constr) {
		BuildTerminalConstraints(solution);
	}

	if (mpc_parameters_->warmstart) {
		ComputeWarmStart(solution);//TODO: Modify the solution or the problem?
	}
}

void QPBuilder::BuildGlobalVelocityReference(const MPCSolution &solution) {
	if (vel_ref_->global.x.rows() != mpc_parameters_->num_samples_state) {
		vel_ref_->global.x.resize(mpc_parameters_->num_samples_state);
		vel_ref_->global.y.resize(mpc_parameters_->num_samples_state);
	}

	double trunk_yaw;
	for (int i = 0; i < mpc_parameters_->num_samples_state; ++i) {
		trunk_yaw = solution.support_states_vec[i+1].yaw;
		vel_ref_->global.x(i) = vel_ref_->local.x(i) * cos(trunk_yaw) - vel_ref_->local.y(i) * sin(trunk_yaw);
		vel_ref_->global.y(i) = vel_ref_->local.x(i) * sin(trunk_yaw) + vel_ref_->local.y(i) * cos(trunk_yaw);
	}
}

void QPBuilder::TransformControlVector(MPCSolution &solution) {
	int ns_c = mpc_parameters_->num_samples_contr;
	int ns_st = mpc_parameters_->num_samples_state;
	int num_steps = solution.support_states_vec.back().step_number;
	int num_unst_modes = 0;
	if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		num_unst_modes = 1;
	}

	const SelectionMatrices &select = preview_->selection_matrices();
	//const CommonMatrixType &rot_mat = preview_->rot_mat();
	//const BodyState &com = robot_->com()->state();
	const CommonVectorType &feet_x_vec = solution.qp_solution_vec.segment(2*(ns_c + num_unst_modes), num_steps);
	const CommonVectorType &feet_y_vec = solution.qp_solution_vec.segment(2*(ns_c + num_unst_modes) + num_steps, num_steps);

	//const CommonVectorType &local_cop_vec_x = solution.qp_solution_vec.segment(0, ns_c);
	//const CommonVectorType &local_cop_vec_y = solution.qp_solution_vec.segment(ns_c + num_unst_modes, ns_c);
	const CommonVectorType &local_contr_vec_x = solution.qp_solution_vec.segment(0, ns_c + num_unst_modes);
	const CommonVectorType &local_contr_vec_y = solution.qp_solution_vec.segment(ns_c + num_unst_modes, ns_c + num_unst_modes);


	//CommonVectorType &global_cop_x_vec = solution.com_prw.cop.x_vec;
	//CommonVectorType &global_cop_y_vec = solution.com_prw.cop.y_vec;

	/*
	int num_samples_interp = 1;
	//New
	int size_contrvec = 2;
	if (mpc_parameters_->interpolate_whole_horizon == true) {
		num_samples_interp = ns_c;
		size_contrvec = ns_c + 1;
	}
	 */
	//New
	int samples_left = mpc_parameters_->GetMPCSamplesLeft(solution.first_coarse_period);
	const CommonMatrixType &input_map_mat = robot_->com()->dynamics_qp_vec()[samples_left].input_map_mat;
	CommonVectorType global_contr_x_vec = input_map_mat * local_contr_vec_x;
	CommonVectorType global_contr_y_vec = input_map_mat * local_contr_vec_y;

	//for (int s = 0; s < num_samples_interp; ++s) {
	// Rotate in local frame: Rx*x - Ry*y;
	//global_cop_x_vec(s) =  rot_mat(s, s) * local_cop_vec_x(s) - rot_mat(s, num_samples + s) * local_cop_vec_y(s);
	//global_cop_x_vec(s) += select.sample_step_cx(s);
	//global_cop_y_vec(s) =  rot_mat(num_samples + s, num_samples + s) * local_cop_vec_y(s) - rot_mat(num_samples + s, s) * local_cop_vec_x(s);
	//global_cop_y_vec(s) += select.sample_step_cy(s);
	//global_cop_x_vec(s) = local_cop_vec_x(s) + select.sample_step_cx(s);
	//global_cop_y_vec(s) = local_cop_vec_y(s) + select.sample_step_cy(s);
	//New
	//global_contr_x_vec(s) = local_contr_vec_x(s) + select.sample_step_cx(s);
	//global_contr_y_vec(s) = local_contr_vec_y(s) + select.sample_step_cy(s);
	//}
	//New: Transform position of ZMP to global
	global_contr_x_vec.head(ns_st) += select.sample_step_cx.block(0, 0, ns_st, 1);
	global_contr_y_vec.head(ns_st) += select.sample_step_cy.block(0, 0, ns_st, 1);

	//global_cop_x_vec += select.sample_step.block(0, 0, num_samples, num_steps) * feet_x_vec;//TODO(performance): Optimize this for first interpolate_whole_horizon == false
	//global_cop_y_vec += select.sample_step.block(0, 0, num_samples, num_steps) * feet_y_vec;
	//New
	global_contr_x_vec.head(ns_st) += select.sample_step.block(0, 0, ns_st, num_steps) * feet_x_vec;
	global_contr_y_vec.head(ns_st) += select.sample_step.block(0, 0, ns_st, num_steps) * feet_y_vec;

	/*
	CommonVectorType state_x(mpc_parameters_->dynamics_order), state_y(mpc_parameters_->dynamics_order);
	if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		Matrix2D state_trans_mat = Matrix2D::Zero();
		state_trans_mat(0, 0) = 1.;
		state_trans_mat(0, 1) = -1./sqrt(kGravity / com.z[0]);
		state_trans_mat(1, 0) = 1.;
		state_trans_mat(1, 1) = 1./sqrt(kGravity / com.z[0]);
		state_x = state_trans_mat * com.x.head(mpc_parameters_->dynamics_order);
		state_y = state_trans_mat * com.y.head(mpc_parameters_->dynamics_order);
	} else {
		state_x = com.x.head(mpc_parameters_->dynamics_order);
		state_y = com.y.head(mpc_parameters_->dynamics_order);
	}
	 */

	//TODO(performance): Optimize this for first interpolate_whole_horizon == false
	// Transform to com motion
	//int samples_left = mpc_parameters_->GetMPCSamplesLeft(solution.first_coarse_period);
	//const LinearDynamicsMatrices &cop_dyn = robot_->com()->dynamics_qp()[samples_left].cop;
	solution.com_prw.control.x_vec.noalias() = global_contr_x_vec;//new - cop_dyn.state_mat * state_x(0);
	//solution.com_prw.control.x_vec.noalias() = copdyn.input_mat_inv * tmp_vec_;
	solution.com_prw.control.y_vec.noalias() = global_contr_y_vec;//new - cop_dyn.state_mat * state_y(0);
	//solution.com_prw.control.y_vec.noalias() = copdyn.input_mat_inv * tmp_vec_;

}

//
// Private methods:
//

void QPBuilder::BuildObjective(const MPCSolution &solution) {
	// Choose the precomputed element depending on the period until next sample
	int samples_left = mpc_parameters_->GetMPCSamplesLeft(solution.first_coarse_period);
	int mat_num = samples_left + mpc_parameters_->penalties.active_mode * mpc_parameters_->GetNumDynamics();
	int num_unst_modes = 0;
	if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		num_unst_modes = 1;
	}

	const BodyState &com = robot_->com()->state();
	const SelectionMatrices &select_mats = preview_->selection_matrices();
	//const CommonMatrixType &rot_mat = preview_->rot_mat();
	//const CommonMatrixType &rot_mat2 = preview_->rot_mat2();
	//const CommonMatrixType &rot_mat2_trans = preview_->rot_mat2_tr();

	QPMatrix &hessian_mat = solver_->hessian_mat();

	int num_steps_previewed = solution.support_states_vec.back().step_number;
	int ns_st = mpc_parameters_->num_samples_state;
	int ns_c = mpc_parameters_->num_samples_contr;

	CommonMatrixType state_x = CommonMatrixType::Zero(mpc_parameters_->dynamics_order - num_unst_modes, 1);
	CommonMatrixType state_y = CommonMatrixType::Zero(mpc_parameters_->dynamics_order - num_unst_modes, 1);
	if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		tmp_mat_ = state_trans_mat_ * com.x.head(mpc_parameters_->dynamics_order);
		tmp_mat2_ = state_trans_mat_ * com.y.head(mpc_parameters_->dynamics_order);

		for (int i = 0; i < mpc_parameters_->dynamics_order - num_unst_modes; i++) {
			state_x(i, 0) = tmp_mat_(i, 0);
			state_y(i, 0) = tmp_mat2_(i, 0);
		}
	} else {
		state_x = com.x.head(mpc_parameters_->dynamics_order);
		state_y = com.y.head(mpc_parameters_->dynamics_order);
	}

	/*
	if (mpc_parameters_->penalties.dcop_online) {
		int i = 0;
		for (; i < mpc_parameters_->num_samples_first_fine_period; i++) {
			dcop_pen_mat_vec_[mat_num](i, i) = mpc_parameters_->penalties.first_contr_moves;
		}
		for (; i < mpc_parameters_->num_samples_first_fine_period + mpc_parameters_->num_samples_first_coarse_period - 1; i++) {
			dcop_pen_mat_vec_[mat_num](i, i) = mpc_parameters_->penalties.second_contr_moves;
		}
	}
	 */

	if (!solver_->do_build_cholesky()) {//TODO: This part is the most costly >50%
		hessian_mat.AddTerm(const_hessian_n_mat_[mat_num], 0, 0);
		hessian_mat.AddTerm(const_hessian_n_mat_[mat_num], ns_c + num_unst_modes, ns_c + num_unst_modes);

		//Online adaptation of control move penalties:
		//--------------------------------------------
		// Works only without input mapping
		/*
		if (mpc_parameters_->penalties.dcop_online) {
			tmp_mat_ = dcop_mat_tr_vec_[mat_num] * dcop_pen_mat_vec_[mat_num] * dcop_mat_vec_[mat_num];
			hessian.AddTerm(tmp_mat_, 0, 0);
			hessian.AddTerm(tmp_mat_, num_samples + num_unst_modes, num_samples + num_unst_modes);
		}
		 */

		//TODO: Unstable modes - adapt rotation
		//CommonMatrixType hessian_mat = hessian().block(0, 0, 2 * num_samples, 2 * num_samples);
		//tmp_mat_.noalias() = rot_mat2 * hessian_mat * rot_mat2_trans;//TODO: Use MTimesRT
		//hessian().block(0, 0, 2 * num_samples, 2 * num_samples) = tmp_mat_;


	} /*else {
		// rotate the cholesky matrix
		CommonMatrixType chol = const_cholesky_[mat_num];
		RotateCholeskyMatrix(chol, rot_mat2);
		hessian.cholesky(chol);
	}*/


	// -----------------------------------------
	// Most of computation time spent from here:
	// -----------------------------------------
	const LinearDynamics &dyn = robot_->com()->dynamics_qp_vec()[samples_left];
	if (num_steps_previewed > 0) {
		//tmp_mat_.noalias() = const_hessian_mat_[matrix_num].block(0, 0, num_samples, num_samples) * select_mats.sample_step;


		//if (mpc_parameters_->penalties.dcop_online) {
		//	tmp_mat2_ += dcop_mat_tr_vec_[mat_num].block(0, 0, num_samples + num_unst_modes, num_samples) * dcop_pen_mat_vec_[mat_num].block(0, 0, num_samples, num_samples) * dcop_mat_vec_[mat_num].block(0, 0, num_samples, num_samples);
		//}


		tmp_mat_.noalias() = u_trans_v_mat_vec_[mat_num] * select_mats.sample_step;


		hessian_mat.AddTerm(tmp_mat_, 0, 2 * (ns_c + num_unst_modes));
		hessian_mat.AddTerm(tmp_mat_, ns_c + num_unst_modes, 2*(ns_c + num_unst_modes) + num_steps_previewed);

		if (!solver_->do_build_cholesky()) {
			hessian_mat.AddTerm(tmp_mat_.transpose(), 2*(ns_c + num_unst_modes), 0);
			hessian_mat.AddTerm(tmp_mat_.transpose(), 2*(ns_c + num_unst_modes) + num_steps_previewed, (ns_c + num_unst_modes));
			// rotate the lower left block
			// TODO(andrei): Rotation can be done before addition
			// TODO: Unstable modes - Adapt rotation
			//CommonMatrixType low_triang_mat = hessian().block(2 * num_samples, 0, 2 * num_steps_previewed, 2 * num_samples);
			//MTimesRT(low_triang_mat, rot_mat2);
			//hessian().block(2 * num_samples, 0, 2 * num_steps_previewed, 2 * num_samples) = low_triang_mat;
		}

		//tmp_mat_.noalias() = select_mats.sample_step_trans * const_hessian_mat_[matrix_num].block(0, 0, num_samples, num_samples) * select_mats.sample_step;

		//if (mpc_parameters_->penalties.dcop_online) {
		//	tmp_mat2_ += dcop_mat_tr_vec_[mat_num].block(0, 0, num_samples + num_unst_modes, num_samples) * dcop_pen_mat_vec_[mat_num].block(0, 0, num_samples, num_samples) * dcop_mat_vec_[mat_num].block(0, 0, num_samples, num_samples);
		//}

		tmp_mat2_.noalias() = v_trans_v_mat_vec_[mat_num] * select_mats.sample_step;
		tmp_mat_.noalias() = select_mats.sample_step_trans * tmp_mat2_;
		hessian_mat.AddTerm(tmp_mat_, 2*(ns_c + num_unst_modes), 2*(ns_c + num_unst_modes));
		hessian_mat.AddTerm(tmp_mat_, 2*(ns_c + num_unst_modes) + num_steps_previewed, 2*(ns_c + num_unst_modes) + num_steps_previewed);

		// rotate the upper right block
		// TODO(efficiency): Rotation can be done before addition
		// TODO: Unstable modes - Adapt rotation
		//CommonMatrixType upp_triang_mat = hessian().block(0, 2*num_samples, 2*num_samples, 2*num_steps_previewed);
		//RTimesM(upp_triang_mat, rot_mat2);
		//hessian().block(0, 2 * num_samples, 2 * num_samples, 2 * num_steps_previewed) = upp_triang_mat;

	}

	// Compute gains:
	// --------------
	//CommonMatrixType inv_hessian = hessian().block(0, 0, num_samples, num_samples).inverse();
	//CommonMatrixType gains_mat = - inv_hessian * (state_variant_[mat_num] /* + curr_cop_variant_[matrix_num] * *last_des_cop_x_*/);
	//double gain_cp = gains_mat(0,0) + gains_mat(0, 1) * 1. / sqrt(kGravity / com.z[0]);
	//std::cout << "gain" << gain_cp << std::endl;

	CommonVectorType objective_tmp_vec_x(ns_st + num_unst_modes),
			objective_tmp_vec_y(ns_st + num_unst_modes),
			objective_vec_x(ns_c + num_unst_modes),
			objective_vec_y(ns_c + num_unst_modes),
			objective_vec(2 * (ns_c + num_unst_modes));//TODO: Make this member

	tmp_mat2_ = state_variant_[mat_num];
	objective_tmp_vec_x = tmp_mat2_ * state_x;
	objective_tmp_vec_y = tmp_mat2_ * state_y;

	tmp_mat2_ = select_variant_[mat_num];
	//if (mpc_parameters_->penalties.dcop_online) {
	//	tmp_mat2_ += dcop_mat_tr_vec_[mat_num].block(0, 0, ns_st + num_unst_modes, ns_st) * dcop_pen_mat_vec_[mat_num].block(0, 0, ns_st, ns_st) * dcop_mat_vec_[mat_num].block(0, 0, ns_st, ns_st);
	//}
	objective_tmp_vec_x += tmp_mat2_ * select_mats.sample_step_cx;
	objective_tmp_vec_y += tmp_mat2_ * select_mats.sample_step_cy;

	objective_tmp_vec_x += ref_variant_vel_[mat_num] * vel_ref_->global.x.head(ns_st);
	objective_tmp_vec_y += ref_variant_vel_[mat_num] * vel_ref_->global.y.head(ns_st);

	objective_tmp_vec_x += ref_variant_pos_[mat_num] * pos_ref_->global.x.head(ns_st);
	objective_tmp_vec_y += ref_variant_pos_[mat_num] * pos_ref_->global.y.head(ns_st);

	objective_tmp_vec_x += ref_variant_cp_[mat_num] * cp_ref_->global.x.head(ns_st);
	objective_tmp_vec_y += ref_variant_cp_[mat_num] * cp_ref_->global.y.head(ns_st);

	//double zx_cur = com.x(0) - com.z(0) / kGravity * com.x(2);
	//double zy_cur = com.y(0) - com.z(0) / kGravity * com.y(2);
	objective_tmp_vec_x += curr_cop_variant_[mat_num] * *last_first_contr_x_;//*last_des_cop_x_;//zx_cur;//
	objective_tmp_vec_y += curr_cop_variant_[mat_num] * *last_first_contr_y_;//*last_des_cop_y_;//zy_cur;//

	//Online adaptation of control move penalties:
	//--------------------------------------------
	/*
	if (mpc_parameters_->penalties.dcop_online) {
		// curr_cop_variant_ online:
		// -------------------------
		CommonVectorType e_vec = CommonVectorType::Zero(ns_st);
		e_vec(0) = mpc_parameters_->penalties.first_contr_moves;
		tmp_vec_ = - dcop_mat_tr_vec_[mat_num].block(0, 0, ns_st + num_unst_modes, ns_st) * e_vec;
		objective_tmp_vec_x += tmp_vec_ * *last_des_cop_x_;//zx_cur;//
		objective_tmp_vec_y += tmp_vec_ * *last_des_cop_y_;//zy_cur;//
	}
	 */

	// Offset from the ankle to the support center for CoP centering:
	// --------------------------------------------------------------
	// TODO: First foot left assumption
	double pos_diff_x = 0.;
	double pos_diff_y = 0.;
	//if (solution.support_states_vec.front().phase == DS) {
	tmp_vec_ = CommonVectorType::Zero(ns_st + num_unst_modes);
	tmp_vec2_ = CommonVectorType::Zero(ns_st + num_unst_modes);
	std::vector<SupportState>::const_iterator ss_it = solution.support_states_vec.begin();
	ss_it++; // Point at the first previewed instant
	int si = 0;
	while (ss_it != solution.support_states_vec.end()) {
		if (ss_it->phase == DS) {
			pos_diff_x = cp_ref_->init[X] - robot_->left_foot()->state().x(0);
			pos_diff_y = cp_ref_->init[Y] - robot_->left_foot()->state().y(0);
		} else {
			pos_diff_x = 0.;
			pos_diff_y = 0.;
			/*if (ss_it->foot == LEFT) {
				pos_diff_y = -mpc_parameters_->cop_off;
			} else {
				pos_diff_y = mpc_parameters_->cop_off;
			}*/
		}
		tmp_vec_[si] = pos_diff_x;
		tmp_vec2_[si] = pos_diff_y;
		si++;
		ss_it++;
	}



	objective_tmp_vec_x -= cop_pen_mat_vec_[mat_num] * tmp_vec_;
	objective_tmp_vec_y -= cop_pen_mat_vec_[mat_num] * tmp_vec2_;
	//}


	// Rotation independent part:
	// --------------------------
	if (num_steps_previewed > 0) {

		tmp_vec_.noalias() = select_mats.sample_step_trans * objective_tmp_vec_x.head(ns_st);
		solver_->objective_vec().Add(tmp_vec_, 2*(ns_c + num_unst_modes));
		tmp_vec_.noalias() = select_mats.sample_step_trans * objective_tmp_vec_y.head(ns_st);
		solver_->objective_vec().Add(tmp_vec_, 2*(ns_c + num_unst_modes) + num_steps_previewed);

		tmp_vec2_.noalias() = v_trans_s_mat_vec_[mat_num] * state_x;
		tmp_vec_.noalias() = select_mats.sample_step_trans * tmp_vec2_;
		solver_->objective_vec().Add(tmp_vec_, 2*(ns_c + num_unst_modes));
		tmp_vec2_.noalias() = v_trans_s_mat_vec_[mat_num] * state_y;
		tmp_vec_.noalias() = select_mats.sample_step_trans * tmp_vec2_;
		solver_->objective_vec().Add(tmp_vec_, 2*(ns_c + num_unst_modes) + num_steps_previewed);

		tmp_vec2_.noalias() = v_trans_vc_mat_vec_[mat_num] * select_mats.sample_step_cx;
		tmp_vec_.noalias() = select_mats.sample_step_trans * tmp_vec2_;
		solver_->objective_vec().Add(tmp_vec_, 2*(ns_c + num_unst_modes));
		tmp_vec2_.noalias() = v_trans_vc_mat_vec_[mat_num] * select_mats.sample_step_cy;
		tmp_vec_.noalias() = select_mats.sample_step_trans * tmp_vec2_;
		solver_->objective_vec().Add(tmp_vec_, 2*(ns_c + num_unst_modes) + num_steps_previewed);

		tmp_vec2_.noalias() = v_trans_ref_mat_vec_[mat_num] * cp_ref_->local.x.head(ns_st);
		tmp_vec_.noalias() = select_mats.sample_step_trans * tmp_vec2_;
		solver_->objective_vec().Add(tmp_vec_, 2*(ns_c + num_unst_modes));
		tmp_vec2_.noalias() = v_trans_ref_mat_vec_[mat_num] * cp_ref_->local.y.head(ns_st);
		tmp_vec_.noalias() = select_mats.sample_step_trans * tmp_vec2_;
		solver_->objective_vec().Add(tmp_vec_, 2*(ns_c + num_unst_modes) + num_steps_previewed);
	}

	// Map input:
	// ----------
	objective_vec_x = dyn.input_map_mat_tr * objective_tmp_vec_x;
	objective_vec_y = dyn.input_map_mat_tr * objective_tmp_vec_y;

	objective_vec_x += u_trans_vc_mat_vec_[mat_num] * select_mats.sample_step_cx.block(0, 0, ns_st, 1);
	objective_vec_y += u_trans_vc_mat_vec_[mat_num] * select_mats.sample_step_cy.block(0, 0, ns_st, 1);

	objective_vec_x += u_trans_s_mat_vec_[mat_num] * state_x;
	objective_vec_y += u_trans_s_mat_vec_[mat_num] * state_y;

	objective_vec_x += u_trans_ref_mat_vec_[mat_num] * cp_ref_->local.x.head(ns_st);
	objective_vec_y += u_trans_ref_mat_vec_[mat_num] * cp_ref_->local.y.head(ns_st);

	objective_vec << objective_vec_x, objective_vec_y; //TODO: Unnecessary if rot_mat half the size
	//gradient_vec = rot_mat * gradient_vec;//TODO: Use RTimesV

	solver_->objective_vec().Add(objective_vec, 0);
}

void QPBuilder::BuildEqualityConstraints(const MPCSolution &solution) {

	if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		BuildCPEqConstraints(solution);
	}

	const Wrench *ff_wrench;
	const SupportState &curr_sup = solution.support_states_vec.front();
	if (curr_sup.foot == RIGHT) {//TODO: Move to RBS
		ff_wrench = &robot_->left_foot()->force_sensor();
	} else {
		ff_wrench = &robot_->right_foot()->force_sensor();
	}
	bool is_ff_incontact = false;
	if (curr_sup.phase == SS && ff_wrench->force_z > mpc_parameters_->ds_force_thresh) {
		is_ff_incontact = true;
	}
	//double plann_margin = 0.05;
	double local_ss_time = current_time_ - curr_sup.start_time;
	//double half_ss_time = mpc_parameters_->period_ss / 2.;
	if (curr_sup.phase == SS && solution.support_states_vec.back().step_number > 0 &&
			((local_ss_time > mpc_parameters_->ffoot_plan_period)/* ||
					(is_ff_incontact && local_ss_time > half_ss_time) ||								//Contact occured
					(local_ss_time > plann_margin && local_ss_time < half_ss_time - plann_margin) ||
					(local_ss_time > half_ss_time + plann_margin)*/
					)
	) {
		BuildFootPosEqConstraints(solution);
	}
	//BuildFootPosEqConstraintsSag(solution);

	//if (solution.is_prev_sol_exist == true &&
	//		mpc_parameters_->period_qpsample > (solution.sampling_times_vec[1] - solution.sampling_times_vec[0]) + kEps) {
	//	BuildCoPEqConstraints(solution);
	//}
}

void QPBuilder::BuildInequalityConstraints(const MPCSolution &solution) {
	//TODO: Create class for variable indices

	int num_steps_previewed = solution.support_states_vec.back().step_number;

	BuildCoPIneqConstraints(solution);
	if (num_steps_previewed > 0) {
		BuildFootPosInequalities(solution);
		BuildFootPosIneqConstraints(solution);

		//BuildFootVelConstraints(solution);
	}
}

void QPBuilder::BuildCPEqConstraints(const MPCSolution &solution) {
	int num_steps_previewed = solution.support_states_vec.back().step_number;
	int ns_c 		= mpc_parameters_->num_samples_contr;
	int ns_st 		= mpc_parameters_->num_samples_state;
	int num_unst_modes 		= 1;
	int num_st_modes		= 1;


	int num_constr = solver_->num_constr();
	solver_->num_constr(num_constr + 2);
	int num_eq_constr = solver_->num_eq_constr();
	solver_->num_eq_constr(num_eq_constr + 2);

	const BodyState &com = robot_->com()->state();
	CommonMatrixType state_x = CommonMatrixType::Zero(mpc_parameters_->dynamics_order - num_st_modes, 1);
	CommonMatrixType state_y = CommonMatrixType::Zero(mpc_parameters_->dynamics_order - num_st_modes, 1);
	tmp_mat_ = state_trans_mat_ * com.x.head(mpc_parameters_->dynamics_order);
	tmp_mat2_ = state_trans_mat_ * com.y.head(mpc_parameters_->dynamics_order);

	for (int i = num_st_modes; i < num_st_modes + num_unst_modes; i++) {
		state_x(i - num_st_modes, 0) = tmp_mat_(i, 0);
		state_y(i - num_st_modes, 0) = tmp_mat2_(i, 0);
	}

	const SelectionMatrices &select_mats = preview_->selection_matrices();

	// Build equality constraints for the unstable modes at end of horizon:
	// --------------------------------------------------------------------
	// Choose the precomputed element depending on the period until next sample
	int samples_left = mpc_parameters_->GetMPCSamplesLeft(solution.first_coarse_period);
	const LinearDynamics &dyn = robot_->com()->dynamics_qp_vec()[samples_left];

	CommonVectorType atimesb_vec(ns_st);
	/*
	for (int col = 0; col < num_samples; col++) {
		atimesb_vec(col) = pow(dyn.d_state_mat_pow_vec[col](1,1), -1) * dyn.d_input_mat_vec[col](1);
	}
	//X:
	// xu_0 < Au^{-N}*\mu_x - \sum Au^{-(j-1)}*Bu*u_j < xu_0
	solver_->constr_mat()()(num_constr, num_samples) = pow(dyn.d_state_mat_pow_vec.back()(1,1), -1);
	solver_->constr_mat().AddTerm(-atimesb_vec.transpose(), num_constr, 0);

	if (num_steps_previewed > 0) {
		tmp_mat_ = -atimesb_vec.transpose() * select_mats.sample_step;
		solver_->constr_mat().AddTerm(tmp_mat_, num_constr, 2*(num_samples + num_unst_modes));
	}

	double vcpc = atimesb_vec.transpose() * select_mats.sample_step_cx;
	solver_->lc_bounds_vec()()(num_constr) = state_x + vcpc;
	solver_->uc_bounds_vec()()(num_constr) = state_x + vcpc;

	//Y:
	// yu_0 < Au^{-N}*\mu_y - \sum Au^{-(j-1)}*Bu*u_j < yu_0
	solver_->constr_mat()()(num_constr + num_unst_modes, 2*num_samples + num_unst_modes) = pow(dyn.d_state_mat_pow_vec.back()(1,1), -1);
	solver_->constr_mat().AddTerm(-atimesb_vec.transpose(), num_constr + num_unst_modes, num_samples + num_unst_modes);

	if (num_steps_previewed > 0) {
		tmp_mat_ = -atimesb_vec.transpose() * select_mats.sample_step;
		solver_->constr_mat().AddTerm(tmp_mat_, num_constr + num_unst_modes, 2*(num_samples + num_unst_modes) + num_steps_previewed);
	}

	vcpc = atimesb_vec.transpose() * select_mats.sample_step_cy;
	solver_->lc_bounds_vec()()(num_constr + num_unst_modes) = state_y + vcpc;
	solver_->uc_bounds_vec()()(num_constr + num_unst_modes) = state_y + vcpc;
	 */

	// Forward in time:
	// ----------------
	double neg_pow_state_mats = 1. / dyn.d_state_mat_pow_vec.at(ns_st - 1)(1,1);
	//double pos_pow_state_mats = dyn.d_state_mat_pow_vec.at(ns_st - 1)(1,1);
	for (int col = 0; col < ns_st - 1; col++) {//TODO: This can be done in DynamicsBuilder
		atimesb_vec(col) = dyn.rev_matrix_prod_vec.at(ns_st - 2 - col)(1,1) * dyn.d_input_mat_vec[col](1);
	}
	atimesb_vec(ns_st - 1) = dyn.d_input_mat_vec.at(ns_st - 1)(1);
	atimesb_vec *= neg_pow_state_mats;

	//X:
	// xu_0 < Au^{N}*\s_0^x + \sum Au^{(j-1)}*Bu*u_j < xu_0
	solver_->constr_mat()()(num_constr, ns_c) = neg_pow_state_mats;	// CP

	// Map input:
	// ----------
	// TODO: Can be done in DynamicsBuilder
	tmp_vec_.noalias() = dyn.input_map_mat_tr.block(0, 0, ns_c, ns_st) * atimesb_vec;
	solver_->constr_mat().AddTerm(-tmp_vec_.transpose(), num_constr, 0);
	if (num_steps_previewed > 0) {
		tmp_mat_ = -atimesb_vec.transpose() * select_mats.sample_step;
		solver_->constr_mat().AddTerm(tmp_mat_, num_constr, 2*(ns_c + num_unst_modes));
	}

	CommonVectorType constant_x = /*pos_pow_state_mats **/ state_x + atimesb_vec.transpose() * select_mats.sample_step_cx.block(0, 0, ns_st, 1);// TODO: Block unnecessary?
	solver_->lc_bounds_vec().Add(constant_x, num_constr);
	solver_->uc_bounds_vec().Add(constant_x, num_constr);

	//Y:
	// yu_0 < Au^{-N}*\mu_y - \sum Au^{-(j-1)}*Bu*u_j < yu_0
	solver_->constr_mat()()(num_constr + num_unst_modes, 2 * ns_c + num_unst_modes) = neg_pow_state_mats;

	solver_->constr_mat().AddTerm(-tmp_vec_.transpose(), num_constr + num_unst_modes, ns_c + num_unst_modes);
	if (num_steps_previewed > 0) {
		tmp_mat_ = -atimesb_vec.transpose() * select_mats.sample_step.block(0, 0, ns_st, num_steps_previewed);// TODO: Block unnecessary?
		solver_->constr_mat().AddTerm(tmp_mat_, num_constr + num_unst_modes, 2*(ns_c + num_unst_modes) + num_steps_previewed);
	}

	CommonVectorType constant_y = /*pos_pow_state_mats **/ state_y + atimesb_vec.transpose() * select_mats.sample_step_cy.block(0, 0, ns_st, 1);// TODO: Block unnecessary?
	solver_->lc_bounds_vec().Add(constant_y, num_constr + num_unst_modes);
	solver_->uc_bounds_vec().Add(constant_y, num_constr + num_unst_modes);
}

void QPBuilder::BuildFootPosEqConstraints(const MPCSolution &solution) {
	assert(solver_->num_constr() == solver_->num_eq_constr());

	int num_steps_previewed = solution.support_states_vec.back().step_number;
	int num_samples 		= mpc_parameters_->num_samples_contr;
	int num_unst_modes 		= 0;
	if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		num_unst_modes = 1;
	}

	int num_constr = solver_->num_constr();
	solver_->num_constr(num_constr + 2);
	int num_eq_constr = solver_->num_eq_constr();
	solver_->num_eq_constr(num_eq_constr + 2);

	//X:
	solver_->constr_mat()()(num_constr, 2*(num_samples + num_unst_modes)) = 1.;
	solver_->lc_bounds_vec().Add(solution.prev_first_foot_x, num_constr);
	solver_->uc_bounds_vec().Add(solution.prev_first_foot_x, num_constr);

	//Y:
	solver_->constr_mat()()(num_constr + 1, 2*(num_samples + num_unst_modes) + num_steps_previewed) = 1.;
	solver_->lc_bounds_vec().Add(solution.prev_first_foot_y, num_constr + 1);
	solver_->uc_bounds_vec().Add(solution.prev_first_foot_y, num_constr + 1);
}

void QPBuilder::BuildFootPosEqConstraintsSag(const MPCSolution &solution) {
	assert(solver_->num_constr() == solver_->num_eq_constr());

	//int num_steps_previewed = solution.support_states_vec.back().step_number;
	int num_samples 		= mpc_parameters_->num_samples_contr;
	int num_unst_modes 		= 0;
	if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		num_unst_modes = 1;
	}

	int num_constr = solver_->num_constr();
	solver_->num_constr(num_constr + 2);
	int num_eq_constr = solver_->num_eq_constr();
	solver_->num_eq_constr(num_eq_constr + 2);

	//X:
	solver_->constr_mat()()(num_constr, 2*(num_samples + num_unst_modes)) = 1.;
	solver_->lc_bounds_vec().Add(robot_->robot_data().right_foot.position[0]/*solution.prev_first_foot_x*/, num_constr);
	solver_->uc_bounds_vec().Add(robot_->robot_data().right_foot.position[0]/*solution.prev_first_foot_x*/, num_constr);

	//Y:
	//solver_->constr_mat()()(num_constr + 1, 2*(num_samples + num_unst_modes) + num_steps_previewed) = 1.;
	//solver_->lc_bounds_vec().Add(solution.prev_first_foot_y, num_constr + 1);
	//solver_->uc_bounds_vec().Add(solution.prev_first_foot_y, num_constr + 1);
}

void QPBuilder::BuildCoPEqConstraints(const MPCSolution &solution) {
	assert(solver_->num_constr() == solver_->num_eq_constr());

	int num_samples 		= mpc_parameters_->num_samples_contr;
	int num_unst_modes 		= 0;
	if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		num_unst_modes = 1;
	}

	int num_constr = solver_->num_constr();
	solver_->num_constr(num_constr + 2);
	int num_eq_constr = solver_->num_eq_constr();
	solver_->num_eq_constr(num_eq_constr + 2);

	//X:
	solver_->constr_mat()()(num_constr, 0) = 1.;
	solver_->lc_bounds_vec().Add(solution.prev_cop_x, num_constr);
	solver_->uc_bounds_vec().Add(solution.prev_cop_x, num_constr);

	//Y:
	solver_->constr_mat()()(num_constr + 1, num_samples + num_unst_modes) = 1.;
	solver_->lc_bounds_vec().Add(solution.prev_cop_y, num_constr + 1);
	solver_->uc_bounds_vec().Add(solution.prev_cop_y, num_constr + 1);
}

void QPBuilder::BuildFootPosInequalities(const MPCSolution &solution) {
	int num_ineqs = 5;//TODO: Hard coded here
	int num_steps = solution.support_states_vec.back().step_number;

	foot_inequalities_.Resize(num_ineqs * num_steps , num_steps);

	std::vector<SupportState>::const_iterator prev_ss_it = solution.support_states_vec.begin();
	++prev_ss_it;//Point at the first previewed instant
	for( int i = 0; i < mpc_parameters_->num_samples_state; ++i ){
		//foot positioning constraints
		if( prev_ss_it->state_changed && prev_ss_it->step_number > 0 && prev_ss_it->phase != DS){

			--prev_ss_it;//Foot polygons are defined with respect to the supporting foot
			robot_->GetConvexHull(hull_, FOOT_HULL, *prev_ss_it);
			hull_.RotateVertices(prev_ss_it->yaw);
			hull_.BuildInequalities(prev_ss_it->foot);
			++prev_ss_it;

			int step_number = (prev_ss_it->step_number - 1);

			foot_inequalities_.x_mat.block( step_number * num_ineqs, step_number, num_ineqs, 1) = hull_.a_vec.segment(0, num_ineqs);
			foot_inequalities_.y_mat.block( step_number * num_ineqs, step_number, num_ineqs, 1) = hull_.b_vec.segment(0, num_ineqs);
			foot_inequalities_.c_vec.segment(step_number * num_ineqs, num_ineqs) = hull_.d_vec.segment(0, num_ineqs);
		}
		++prev_ss_it;
	}
}

void QPBuilder::BuildFootPosIneqConstraints(const MPCSolution &solution) {
	int num_steps_previewed = solution.support_states_vec.back().step_number;
	const SelectionMatrices &select = preview_->selection_matrices();
	int ns_c = mpc_parameters_->num_samples_contr;
	int num_unst_modes = 0;
	if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		num_unst_modes = 1;
	}

	int num_constr = solver_->num_constr();
	solver_->num_constr(num_constr + 5*num_steps_previewed);

	tmp_mat_.noalias() = foot_inequalities_.x_mat * select.Vf;
	solver_->constr_mat().AddTerm(tmp_mat_,  num_constr,  2*(ns_c + num_unst_modes) );

	tmp_mat_.noalias() = foot_inequalities_.y_mat * select.Vf;
	solver_->constr_mat().AddTerm(tmp_mat_,  num_constr, 2*(ns_c + num_unst_modes) + num_steps_previewed);

	solver_->lc_bounds_vec().Add(foot_inequalities_.c_vec, num_constr);

	tmp_vec_.noalias() =  foot_inequalities_.x_mat * select.VcfX;
	tmp_vec_ += foot_inequalities_.y_mat * select.VcfY;
	solver_->lc_bounds_vec().Add(tmp_vec_,  num_constr);

	solver_->uc_bounds_vec()().segment(num_constr, tmp_vec_.size()).fill(kInf);
}

void QPBuilder::BuildFootVelIneqConstraints(const MPCSolution &solution) {
	assert(robot_->robot_data().max_foot_vel > kEps);

	double raise_time = 0.05;//TODO: Hard coded value has to come from robot_
	double time_left = solution.support_states_vec.front().time_limit - raise_time - current_time_;
	double max_vel = robot_->robot_data().max_foot_vel;

	int num_steps_previewed = solution.support_states_vec.back().step_number;
	int x_var_pos = mpc_parameters_->num_samples_contr;
	int y_var_pos = mpc_parameters_->num_samples_contr + num_steps_previewed;

	const BodyState *flying_foot;
	const SupportState &current_support = solution.support_states_vec.front();
	if (current_support.foot == LEFT) {
		flying_foot = &robot_->right_foot()->state();
	} else {
		flying_foot = &robot_->left_foot()->state();
	}

	double upper_limit_x = max_vel * time_left + flying_foot->x(0);
	double upper_limit_y = max_vel * time_left + flying_foot->y(0);
	solver_->uv_bounds_vec().Add(upper_limit_x, x_var_pos);
	solver_->uv_bounds_vec().Add(upper_limit_y, y_var_pos);
	double lower_limit_x = -max_vel * time_left + flying_foot->x(0);
	double lower_limit_y = -max_vel * time_left + flying_foot->y(0);
	solver_->lv_bounds_vec().Add(lower_limit_x, x_var_pos);
	solver_->lv_bounds_vec().Add(lower_limit_y, y_var_pos);
}

void QPBuilder::BuildCoPIneqConstraints(const MPCSolution &solution) {
	int ns_st = mpc_parameters_->num_samples_state;
	int ns_c = mpc_parameters_->num_samples_contr;
	int num_unst_modes = 0;//TODO: Define outside
	if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		num_unst_modes = 1;
	}
	std::vector<SupportState>::const_iterator ss_it = solution.support_states_vec.begin();
	//const SupportState &curr_sup = solution.support_states_vec.front();

	robot_->GetConvexHull(hull_, COP_HULL, *ss_it);

	tmp_vec_.resize(2 * ns_st);
	tmp_vec2_.resize(2 * ns_st);


	// Verify if flying foot entered contact:
	// --------------------------------------
	const Wrench *ff_wrench;
	const Wrench *sf_wrench;
	if (ss_it->foot == RIGHT) {
		ff_wrench = &robot_->left_foot()->force_sensor();
		sf_wrench = &robot_->right_foot()->force_sensor();
	} else {
		ff_wrench = &robot_->right_foot()->force_sensor();
		sf_wrench = &robot_->left_foot()->force_sensor();
	}
	bool is_ff_incontact = false;
	if (ss_it->phase == SS && ff_wrench->force_z > mpc_parameters_->ds_force_thresh) {
		is_ff_incontact = true;
	}
	bool is_cur_sf_incontact = false;
	if (ss_it->phase == SS && sf_wrench->force_z > mpc_parameters_->ds_force_thresh) {
		is_cur_sf_incontact = true;
	}
	double lift_off_margin = ss_it->start_time + mpc_parameters_->period_ss / 2.;

	// Has half of ds phase passed?
	std::vector<double>::const_iterator st_it = solution.sampling_times_vec.begin();

	// Foot distance:
	// --------------
	double dist_feet_y = robot_->left_foot()->state().y[POSITION] - robot_->right_foot()->state().y[POSITION];

	// Compose bounds vector:
	// ----------------------
	bool is_half_ds_passed = false;
	++st_it; // Points at the first previewed instant
	++ss_it; // Points at the first previewed instant
	for (int i = 0; i < ns_st; ++i) {
		if (ss_it->state_changed) {
			robot_->GetConvexHull(hull_, COP_HULL, *ss_it);
		}
		// Has half of transitional ds phase passed?
		if (*st_it < ss_it->start_time + 2 * mpc_parameters_->period_trans_ds() - kEps) {
			is_half_ds_passed = false;
		} else {
			is_half_ds_passed = true;
		}
		// X local
		tmp_vec_(i)  = min(hull_.x_vec(0), hull_.x_vec(3));
		tmp_vec2_(i) = max(hull_.x_vec(0), hull_.x_vec(3));

		// Y local
		if ((solution.sampling_times_vec.front() > lift_off_margin && is_ff_incontact && ss_it->step_number == 0)
				|| (!is_half_ds_passed)) {// Contact occured or half of ds phase not passed //TODO: Hard coded here
			if (ss_it->foot == LEFT) {
				tmp_vec_(ns_st + i) = /*min(hull_.y_vec(0), hull_.y_vec(1))*/ - dist_feet_y - max(hull_.y_vec(0), hull_.y_vec(1));
				tmp_vec2_(ns_st + i)= max(hull_.y_vec(0), hull_.y_vec(1));
			} else {
				tmp_vec_(ns_st + i) = min(hull_.y_vec(0), hull_.y_vec(1));
				tmp_vec2_(ns_st + i)= /*max(hull_.y_vec(0), hull_.y_vec(1))*/ - min(hull_.y_vec(0), hull_.y_vec(1)) + dist_feet_y;
			}
		} else {
			if (!is_cur_sf_incontact && ss_it->step_number == 0) {//DS constraints for cur. sup. if NO contact occured
				if (ss_it->foot == LEFT) {
					tmp_vec_(ns_st + i) = /*min(hull_.y_vec(0), hull_.y_vec(1))*/ - dist_feet_y - max(hull_.y_vec(0), hull_.y_vec(1));
					tmp_vec2_(ns_st + i)= max(hull_.y_vec(0), hull_.y_vec(1));
				} else {
					tmp_vec_(ns_st + i) = min(hull_.y_vec(0), hull_.y_vec(1));
					tmp_vec2_(ns_st + i)= /*max(hull_.y_vec(0), hull_.y_vec(1))*/ - min(hull_.y_vec(0), hull_.y_vec(1)) + dist_feet_y;
				}
			} else {//SS constraints (normal functioning)
				//tmp_vec_(ns_st + i) = min(hull_.y_vec(0), hull_.y_vec(1));
				//tmp_vec2_(ns_st + i)= max(hull_.y_vec(0), hull_.y_vec(1));
				// Y local
				if (ss_it->foot == LEFT) {
					tmp_vec_(ns_st + i) = min(hull_.y_vec(0), hull_.y_vec(1));// - dist_feet_y /*- max(hull_.y_vec(0), hull_.y_vec(1))*/;
					tmp_vec2_(ns_st + i)= max(hull_.y_vec(0), hull_.y_vec(1));
				} else {
					tmp_vec_(ns_st + i) = min(hull_.y_vec(0), hull_.y_vec(1));
					tmp_vec2_(ns_st + i)= max(hull_.y_vec(0), hull_.y_vec(1));// /*- min(hull_.y_vec(0), hull_.y_vec(1))*/ + dist_feet_y;
				}
			}
		}

		if (ss_it->step_number > 0) {
			if (ss_it->foot == LEFT) {
				tmp_vec2_(ns_st + i) -= 0.02;//0.045
			} else {
				tmp_vec_(ns_st + i) += 0.02;//0.045
			}

		}

		++ss_it;
		++st_it;
	}


	// Map constraints:
	// ----------------
	const CommonMatrixType &input_map_mat = robot_->com()->dynamics_qp_vec().front().input_map_mat;
	tmp_vec3_.setConstant(2 * ns_c, -kInf);
	tmp_vec4_.setConstant(2 * ns_c, kInf);
	// X:
	tmp_vec3_[0] 	= tmp_vec_[0];
	tmp_vec4_[0] 	= tmp_vec2_[0];
	// Y:
	tmp_vec3_[ns_c] 	= tmp_vec_[ns_st];
	tmp_vec4_[ns_c] 	= tmp_vec2_[ns_st];
	int row = 0;
	int col = 0;
	for (int i = 0; i < ns_st; i++) {
		if (input_map_mat(row, col) < kEps) {
			col++;
		}
		// X:
		tmp_vec3_[col] = max(tmp_vec3_[col], tmp_vec_[i]);
		tmp_vec4_[col] = min(tmp_vec4_[col], tmp_vec2_[i]);
		// Y:
		tmp_vec3_[ns_c + col] = max(tmp_vec3_[ns_c + col], tmp_vec_[ns_st + i]);
		tmp_vec4_[ns_c + col] = min(tmp_vec4_[ns_c + col], tmp_vec2_[ns_st + i]);

		row++;
	}
	//std::cout << "tmp_vec3y_: " << tmp_vec3_.tail(ns_c).transpose() << std::endl;
	//std::cout << "tmp_vec3_: " << tmp_vec3_.transpose() << std::endl;
	//std::cout << "tmp_vec4_: " << tmp_vec4_.transpose() << std::endl;
	/*if (solution.first_coarse_period < mpc_parameters_->period_actsample + kEps ||
			fabs(solution.first_coarse_period - mpc_parameters_->period_qpsample) < kEps) {
		if (curr_sup.foot == LEFT) {
			tmp_vec4_(ns_c) *= 4;
		} else {
			tmp_vec3_(ns_c) *= 4;
		}
	}*/

	// Fill QP:
	// --------
	// X:
	int first_row = 0;
	solver_->lv_bounds_vec().Set(tmp_vec3_.head(ns_c), first_row);
	solver_->uv_bounds_vec().Set(tmp_vec4_.head(ns_c), first_row);
	// Y:
	first_row = ns_c + num_unst_modes;
	solver_->lv_bounds_vec().Set(tmp_vec3_.tail(ns_c), first_row);
	solver_->uv_bounds_vec().Set(tmp_vec4_.tail(ns_c), first_row);
}

void QPBuilder::BuildTerminalConstraints(const MPCSolution &solution) {

	int num_steps_previewed = solution.support_states_vec.back().step_number;
	int num_samples 		= mpc_parameters_->num_samples_contr;
	int num_unst_modes = 0;
	if (mpc_parameters_->formulation == DECOUPLED_MODES) {
		num_unst_modes = 1;
	}

	int num_constr = solver_->num_constr();
	solver_->num_constr(num_constr + 2);

	const SupportState &final_sup_state = solution.support_states_vec.back();
	robot_->GetConvexHull(hull_, COP_HULL, final_sup_state);
	double min_x = min(hull_.x_vec(0), hull_.x_vec(3));
	double max_x = max(hull_.x_vec(0), hull_.x_vec(3));
	double min_y = min(hull_.y_vec(0), hull_.y_vec(1));
	double max_y = max(hull_.y_vec(0), hull_.y_vec(1));

	// Terminal equality constraint on \mu
	// X:
	solver_->lv_bounds_vec()()(num_samples) = 0.;
	solver_->uv_bounds_vec()()(num_samples) = 0.;
	// Y:
	solver_->lv_bounds_vec()()(2*num_samples + num_unst_modes) = 0.;
	solver_->uv_bounds_vec()()(2*num_samples + num_unst_modes) = 0.;

	// Terminal inequality constraint on \mu
	// cop_min < cp - Vp - Vcpc < cop_max
	const SelectionMatrices &select = preview_->selection_matrices();
	// X:
	solver_->constr_mat()()(num_constr, num_samples) = 1.;	// Capture point x
	if (num_steps_previewed > 0) {
		solver_->constr_mat()()(num_constr, 2*(num_samples + num_unst_modes) + num_steps_previewed - 1) = -1.;	// Last foot x
	}
	solver_->lc_bounds_vec()()(num_constr) = min_x + select.sample_step_cx(num_samples - 1);
	solver_->uc_bounds_vec()()(num_constr) = max_x + select.sample_step_cx(num_samples - 1);
	// Y:
	solver_->constr_mat()()(num_constr + 1, 2*num_samples + num_unst_modes) = 1.; // Capture point y
	if (num_steps_previewed > 0) {
		solver_->constr_mat()()(num_constr + 1, 2*(num_samples + num_unst_modes) + 2*num_steps_previewed - 1) = -1.; // Last foot y
	}
	solver_->lc_bounds_vec()()(num_constr + 1) = min_y + select.sample_step_cy(num_samples - 1);
	solver_->uc_bounds_vec()()(num_constr + 1) = max_y + select.sample_step_cy(num_samples - 1);
}

void QPBuilder::ComputeWarmStart(MPCSolution &solution) {
	//TODO: This function is not reviewed. Possibly devide this function in two parts:
	// Initialize:
	// -----------
	int num_steps = solution.support_states_vec.back().step_number;
	int num_steps_max = mpc_parameters_->num_samples_contr;

	int nbFC = 5;// Number of foot constraints per step TODO: can be read?
	int num_samples = mpc_parameters_->num_samples_contr;
	solution.initial_solution.resize(4*num_samples + 4*num_steps);//TODO: 2*num_samples+2*num_steps
	//TODO: resize necessary?

	// Preview active set:
	// -------------------
	int size = solution.init_active_set.rows();//TODO: size of what?
	Eigen::VectorXi initialConstraintTmp = solution.init_active_set;//TODO: Copy not necessary for shifting
	//double TimeFactor = solution.support_states_vec[1].sampleWeight;//TODO: TimeFactor? sampleWeight??
	int shiftCtr = 0;
	//if (fabs(TimeFactor-1.) < kEps) {
	//	shiftCtr = 1;//if sampleWeight == 1
	//}
	if (size >= 2*num_samples){//TODO: Verification wouldn't be necessary without copying
		// Shift active set by
		solution.init_active_set.segment(0,         num_samples-1) = initialConstraintTmp.segment(shiftCtr,    num_samples-1);
		solution.init_active_set.segment(num_samples, num_samples-1) = initialConstraintTmp.segment(shiftCtr+num_samples, num_samples-1);

		// New final ZMP elements are old final elements
		solution.init_active_set(  num_samples-1) = initialConstraintTmp(  num_samples-1);
		solution.init_active_set(2 * num_samples-1) = initialConstraintTmp(2*num_samples-1);

		// Foot constraints are not shifted
		solution.init_active_set.segment(2 * num_samples, nbFC*num_steps)=
				initialConstraintTmp.segment (2 * num_samples, nbFC*num_steps);
		solution.init_active_set.segment(2 * num_samples + nbFC*num_steps, nbFC * (num_steps_max - num_steps))=
				initialConstraintTmp.segment (2 * num_samples + nbFC * num_steps, nbFC * (num_steps_max - num_steps));
	} else {
		solution.init_active_set = Eigen::VectorXi::Zero(2*num_samples+(4+nbFC)*num_steps_max);//TODO: Why this value?
	}

	//TODO: Checked until here.

	// Compute feasible initial ZMP and foot positions:
	// ------------------------------------------------
	std::vector<SupportState>::iterator previewed_ss_it = solution.support_states_vec.begin();
	++previewed_ss_it;//Point at the first previewed support state

	SupportState current_support = solution.support_states_vec.front();
	// if in transition phase
	if (previewed_ss_it->state_changed){
		current_support = *previewed_ss_it;
	}
	int j = 0;

	double shiftx,shifty;
	bool noActiveConstraints;
	for (int i = 0; i<num_samples; i++){
		// Get COP convex hull for current support
		robot_->GetConvexHull(cop_hull_edges_, COP_HULL, *previewed_ss_it);
		cop_hull_edges_.BuildInequalities(previewed_ss_it->foot);

		// Check if the support foot has changed
		if (previewed_ss_it->state_changed && previewed_ss_it->step_number>0){

			// Get feet convex hull for current support
			previewed_ss_it--;
			robot_->GetConvexHull(foot_hull_edges_, FOOT_HULL, *previewed_ss_it);
			foot_hull_edges_.BuildInequalities(previewed_ss_it->foot);
			previewed_ss_it++;

			// Place the foot on active constraints
			shiftx = shifty = 0;
			noActiveConstraints = true;
			for(int k = 0; k < nbFC; ++k){
				if (solution.init_active_set(k+2*num_samples+j*nbFC)!=0){
					int k2 = (k+1)%5; // k(4) = k(0)
					if (solution.init_active_set(k2+2*num_samples+j*nbFC)!=0){
						shiftx=foot_hull_edges_.x_vec(k2);
						shifty=foot_hull_edges_.y_vec(k2);
					}else{
						shiftx = (foot_hull_edges_.x_vec(k) + foot_hull_edges_.x_vec(k2)) / 2.;
						shifty = (foot_hull_edges_.y_vec(k) + foot_hull_edges_.y_vec(k2)) / 2.;
					}
					noActiveConstraints = false;
					break;
				}
			}
			if (noActiveConstraints){
				shiftx = (foot_hull_edges_.x_vec(4) + foot_hull_edges_.x_vec(0)) / 2.;
				shifty = (foot_hull_edges_.y_vec(4) + foot_hull_edges_.y_vec(2)) / 2.;
			}

			current_support.x += shiftx;
			current_support.y += shifty;

			// Set the new position into initial solution vector
			solution.initial_solution(2*num_samples+j) = current_support.x;
			solution.initial_solution(2*num_samples+num_steps+j) = current_support.y;
			++j;
		}
		// Place the ZMP on active constraints
		shiftx=shifty=0;
		noActiveConstraints = true;
		int k1=-1;
		int k2=-1;
		if (solution.init_active_set(0+i*2)==1){
			if (solution.init_active_set(num_samples+i*2)==1){
				k2=1;
				noActiveConstraints = false;
			} else if (solution.init_active_set(num_samples+i*2)==2){
				k2=0;
				noActiveConstraints = false;
			} else if (solution.init_active_set(num_samples+i*2)==0){
				k1=0;
				k2=1;
				noActiveConstraints = false;
			}
		}else if (solution.init_active_set(0+i*2)==2){
			if (solution.init_active_set(num_samples+i*2)==1){
				k2=2;
				noActiveConstraints = false;
			}else if (solution.init_active_set(num_samples+i*2)==2){
				k2=3;
				noActiveConstraints = false;
			}else if (solution.init_active_set(num_samples+i*2)==0){
				k1=3;
				k2=2;
				noActiveConstraints = false;
			}
		}else if (solution.init_active_set(num_samples+i*2)==1){
			k1=2;
			k2=1;
			noActiveConstraints = false;
		}else if (solution.init_active_set(num_samples+i*2)==2){
			k1=0;
			k2=3;
			noActiveConstraints = false;
		}

		if (!noActiveConstraints){
			if (k1!=-1){
				shiftx=(cop_hull_edges_.x_vec[k1]+cop_hull_edges_.x_vec[k2])/2;
				shifty=(cop_hull_edges_.y_vec[k1]+cop_hull_edges_.y_vec[k2])/2;
			}else{
				shiftx=cop_hull_edges_.x_vec[k2];
				shifty=cop_hull_edges_.y_vec[k2];
			}
		}

		solution.initial_solution(i) = shiftx;
		solution.initial_solution(num_samples+i) = shifty;
		++previewed_ss_it;

	}

}

