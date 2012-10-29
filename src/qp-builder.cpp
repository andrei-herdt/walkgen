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
		RealClock *clock)
:preview_(preview)
,solver_(solver)
,robot_(robot)
,pos_ref_(pos_ref)
,vel_ref_(vel_ref)
,cp_ref_(cp_ref)
,mpc_parameters_(mpc_parameters)
,tmp_vec_(1)
,tmp_vec2_(1)
,tmp_mat_(1,1)
,tmp_mat2_(1,1)
,current_time_(0.)
,clock_(clock)
{}

QPBuilder::~QPBuilder() {}


void QPBuilder::PrecomputeObjective() {
	//TODO: The following has already been done in Walkgen::Init()
	// Define order of variables:
	// --------------------------
	int num_samples = mpc_parameters_->num_samples_horizon;
	Eigen::VectorXi order(2 * num_samples);
	for (int i = 0; i < num_samples; ++i) {
		order(i) = 2 * i;
		order(i + num_samples) = 2 * i + 1;
	}
	QPMatrix chol(2 * num_samples, 2 * num_samples);
	chol.row_indices(order);
	chol.column_indices(order);

	int num_modes = mpc_parameters_->weights.control.size();
	int num_recomp = mpc_parameters_->GetNumRecomputations();
	const_hessian_mat_.resize(num_recomp * num_modes);
	const_hessian_n_mat_.resize(num_recomp * num_modes);
	const_cholesky_.resize(num_recomp * num_modes);
	state_variant_.resize(num_recomp * num_modes);
	select_variant_.resize(num_recomp * num_modes);
	ref_variant_vel_.resize(num_recomp * num_modes);
	ref_variant_pos_.resize(num_recomp * num_modes);
	ref_variant_cp_.resize(num_recomp * num_modes);

	// Set R:
	// ------
	CommonMatrixType contr_weighting_mat;
	contr_weighting_mat.setIdentity(num_samples, num_samples);
	if (mpc_parameters_->dynamics_order == SECOND_ORDER) {
		for (int row = 1; row < num_samples; row++) {
			//contr_weighting_mat(row, row - 1) = -1.;
		}
	}

	// Set gains for the pid mode:
	if (mpc_parameters_->is_pid_mode) {
		mpc_parameters_->weights.control[0] = 1.;
		mpc_parameters_->weights.control[1] = 1.;
		mpc_parameters_->weights.cp[0] = 1.;
		mpc_parameters_->weights.cp[1] = 1.;
	}

	// Precompute:
	// -----------
	CommonMatrixType hessian_mat(num_samples, num_samples);
	for (int mode_num = 0; mode_num < num_modes; ++mode_num) {
		for (double first_period = mpc_parameters_->period_qpsample; first_period > kEps; first_period -= mpc_parameters_->period_mpcsample) {
			int samples_left = mpc_parameters_->GetMPCSamplesLeft(first_period);
			int mat_num = samples_left + mode_num * num_recomp;
			//std::cout << "mat_num: " << mat_num << std::endl;
			//std::cout << "first_period: " << first_period << std::endl;

			const LinearDynamicsMatrices &pos_dyn = robot_->com()->dynamics_qp()[samples_left].pos;
			const LinearDynamicsMatrices &vel_dyn = robot_->com()->dynamics_qp()[samples_left].vel;
			const LinearDynamicsMatrices &cop_dyn = robot_->com()->dynamics_qp()[samples_left].cop;
			const LinearDynamicsMatrices &cp_dyn = robot_->com()->dynamics_qp()[samples_left].cp;

			// Q = beta*Uz^(-T)*Uv^T*Uv*Uz^(-1)
			tmp_mat_.noalias() = cop_dyn.input_mat_inv_tr * vel_dyn.input_mat_tr * vel_dyn.input_mat * cop_dyn.input_mat_inv;
			hessian_mat = mpc_parameters_->weights.vel[mode_num] * tmp_mat_;

			// Q += gamma*Uz^(-T)*Uz^(-1)
			tmp_mat_.noalias() = cop_dyn.input_mat_inv_tr * cop_dyn.input_mat_inv;
			hessian_mat += mpc_parameters_->weights.control[mode_num] * tmp_mat_;
			// Q += delta*Uz^(-T)*Up^T*Up*Uz^(-1)
			tmp_mat_.noalias() = cop_dyn.input_mat_inv_tr * pos_dyn.input_mat_tr * pos_dyn.input_mat * cop_dyn.input_mat_inv;
			hessian_mat +=  mpc_parameters_->weights.pos[mode_num] * tmp_mat_;
			// Q += epsilon*Uz^(-T)*Uxi^T*Uxi*Uz^(-1)
			tmp_mat_.noalias() = cop_dyn.input_mat_inv_tr * cp_dyn.input_mat_tr * cp_dyn.input_mat * cop_dyn.input_mat_inv;
			hessian_mat +=  mpc_parameters_->weights.cp[mode_num] * tmp_mat_;

			const_hessian_mat_[mat_num] = hessian_mat;
			// Q += gamma*I
			const_hessian_n_mat_[mat_num] = hessian_mat + mpc_parameters_->weights.cop[mode_num] * contr_weighting_mat;

			chol.Reset(0.);
			chol.AddTerm(const_hessian_n_mat_[mat_num], 0, 0);
			chol.AddTerm(const_hessian_n_mat_[mat_num], num_samples, num_samples);

			const_cholesky_[mat_num] = chol.cholesky();

			// beta*Uz^(-T)*Uv*(Sv - Uv*Uz^(-1)*Sz)
			tmp_mat_.noalias() = vel_dyn.state_mat - vel_dyn.input_mat * cop_dyn.input_mat_inv * cop_dyn.state_mat;
			state_variant_[mat_num] = cop_dyn.input_mat_inv_tr * vel_dyn.input_mat_tr * mpc_parameters_->weights.vel[mode_num] * tmp_mat_;
			// beta*Uz^(-T)*Up*(Sp - Up*Uz^(-1)*Sz)
			tmp_mat_.noalias() = pos_dyn.state_mat - pos_dyn.input_mat * cop_dyn.input_mat_inv * cop_dyn.state_mat;
			state_variant_[mat_num] += cop_dyn.input_mat_inv_tr * pos_dyn.input_mat_tr * mpc_parameters_->weights.pos[mode_num] * tmp_mat_;
			// epsilon*Uz^(-T)*Uxi*(Sp - Uxi*Uz^(-1)*Sz)
			tmp_mat_.noalias() = cp_dyn.state_mat - cp_dyn.input_mat * cop_dyn.input_mat_inv * cop_dyn.state_mat;
			state_variant_[mat_num] += cop_dyn.input_mat_inv_tr * cp_dyn.input_mat_tr * mpc_parameters_->weights.cp[mode_num] * tmp_mat_;
			// - alpha*Uz^(-T)*Uz^(-1)*Sz
			state_variant_[mat_num] -= cop_dyn.input_mat_inv_tr * mpc_parameters_->weights.control[mode_num] * cop_dyn.input_mat_inv * cop_dyn.state_mat;

			// alpha*Uz^(-T)*Uz^(-1)
			select_variant_[mat_num]  = cop_dyn.input_mat_inv_tr * mpc_parameters_->weights.control[mode_num] * cop_dyn.input_mat_inv;
			// beta*Uz^(-T)*Uv^T*Uv*Uz^(-1)
			select_variant_[mat_num] += cop_dyn.input_mat_inv_tr * vel_dyn.input_mat_tr * mpc_parameters_->weights.vel[mode_num] * vel_dyn.input_mat * cop_dyn.input_mat_inv;
			// delta*Uz^(-T)*Up^T*Up*Uz^(-1)
			select_variant_[mat_num] += cop_dyn.input_mat_inv_tr * pos_dyn.input_mat_tr * mpc_parameters_->weights.pos[mode_num] * pos_dyn.input_mat * cop_dyn.input_mat_inv;
			// epsilon*Uz^(-T)*Uxi^T*Uxi*Uz^(-1)
			select_variant_[mat_num] += cop_dyn.input_mat_inv_tr * cp_dyn.input_mat_tr * mpc_parameters_->weights.cp[mode_num] * cp_dyn.input_mat * cop_dyn.input_mat_inv;

			// - beta*Uz^(-T)*Uv^T
			ref_variant_vel_[mat_num] = -cop_dyn.input_mat_inv_tr * vel_dyn.input_mat_tr * mpc_parameters_->weights.vel[mode_num];
			// - delta*Uz^(-T)*Up^T
			ref_variant_pos_[mat_num] = -cop_dyn.input_mat_inv_tr * pos_dyn.input_mat_tr * mpc_parameters_->weights.pos[mode_num];
			// - epsilon*Uz^(-T)*Uxi^T
			ref_variant_cp_[mat_num] = -cop_dyn.input_mat_inv_tr * cp_dyn.input_mat_tr * mpc_parameters_->weights.cp[mode_num];
		}
	}
}

void QPBuilder::BuildProblem(MPCSolution &solution) {
	// DIMENSION OF QP:
	// ----------------
	int num_variables = 2 * mpc_parameters_->num_samples_horizon +			// com
			2 * solution.support_states_vec.back().step_number;				// Foot placement
	int num_constr = 5 * solution.support_states_vec.back().step_number;	// Foot placement
	solver_->num_var(num_variables);
	solver_->num_constr(num_constr);

	BuildObjective(solution);
	BuildConstraints(solution);
	if (mpc_parameters_->warmstart) {
		ComputeWarmStart(solution);//TODO: Modify the solution?
	}
}

void QPBuilder::BuildGlobalVelocityReference(const MPCSolution &solution) {
	if (vel_ref_->global.x.rows() != mpc_parameters_->num_samples_horizon) {
		vel_ref_->global.x.resize(mpc_parameters_->num_samples_horizon);
		vel_ref_->global.y.resize(mpc_parameters_->num_samples_horizon);
	}

	double trunk_yaw;
	for (int i = 0; i < mpc_parameters_->num_samples_horizon; ++i) {
		trunk_yaw = solution.support_states_vec[i+1].yaw;
		vel_ref_->global.x(i) = vel_ref_->local.x(i) * cos(trunk_yaw) - vel_ref_->local.y(i) * sin(trunk_yaw);
		vel_ref_->global.y(i) = vel_ref_->local.x(i) * sin(trunk_yaw) + vel_ref_->local.y(i) * cos(trunk_yaw);
	}
}

void QPBuilder::TransformControlVector(MPCSolution &solution) {
	int num_samples = mpc_parameters_->num_samples_horizon;
	int num_steps = solution.support_states_vec.back().step_number;

	const SelectionMatrices &select = preview_->selection_matrices();
	const CommonMatrixType &rot_mat = preview_->rot_mat();
	const BodyState &com = robot_->com()->state();

	const CommonVectorType &local_cop_vec = solution.qp_solution_vec;
	const CommonVectorType feet_x_vec = solution.qp_solution_vec.segment(2 * num_samples, num_steps);
	const CommonVectorType feet_y_vec = solution.qp_solution_vec.segment(2 * num_samples + num_steps, num_steps);

	CommonVectorType &global_cop_x_vec = solution.com_prw.cop.x_vec;
	CommonVectorType &global_cop_y_vec = solution.com_prw.cop.y_vec;

	int num_samples_interp = 1;
	if (mpc_parameters_->interpolate_whole_horizon == true) {
		num_samples_interp = num_samples;
	}

	for (int s = 0; s < num_samples_interp; ++s) {
		// Rotate in local frame: Rx*x - Ry*y;
		global_cop_x_vec(s) =  rot_mat(s, s) * local_cop_vec(s) - rot_mat(s, num_samples + s) * local_cop_vec(num_samples + s);
		global_cop_x_vec(s) += select.sample_step_cx(s);
		global_cop_y_vec(s) =  rot_mat(num_samples + s, num_samples + s) * local_cop_vec(num_samples + s) - rot_mat(num_samples + s, s) * local_cop_vec(s);
		global_cop_y_vec(s) += select.sample_step_cy(s);
	}

	// CoP position in the global frame
	//for (int step = 0; step < num_steps; step++) {
	//
	//}
	global_cop_x_vec += select.sample_step * feet_x_vec;//TODO(performance): Optimize this for first interpolate_whole_horizon == false
	global_cop_y_vec += select.sample_step * feet_y_vec;

	CommonVectorType state_x(mpc_parameters_->dynamics_order), state_y(mpc_parameters_->dynamics_order);
	for (int i = 0; i < mpc_parameters_->dynamics_order; i++) {
		state_x(i) = com.x(i);
		state_y(i) = com.y(i);
	}

	//TODO(performance): Optimize this for first interpolate_whole_horizon == false
	// Transform to com motion
	int samples_left = mpc_parameters_->GetMPCSamplesLeft(solution.sampling_times_vec[1] - solution.sampling_times_vec[0]);
	const LinearDynamicsMatrices &copdyn = robot_->com()->dynamics_qp()[samples_left].cop;
	tmp_vec_.noalias() = global_cop_x_vec - copdyn.state_mat * state_x;
	solution.com_prw.control.x_vec.noalias() = copdyn.input_mat_inv * tmp_vec_;
	tmp_vec_.noalias() = global_cop_y_vec - copdyn.state_mat * state_y;
	solution.com_prw.control.y_vec.noalias() = copdyn.input_mat_inv * tmp_vec_;

}

//
// Private methods:
//
void QPBuilder::BuildObjective(const MPCSolution &solution) {
	// Choose the precomputed element depending on the period until next sample
	int samples_left = mpc_parameters_->GetMPCSamplesLeft(solution.sampling_times_vec[1] - solution.sampling_times_vec[0]);
	samples_left += mpc_parameters_->weights.active_mode * mpc_parameters_->GetNumRecomputations();

	const BodyState &com = robot_->com()->state();
	const SelectionMatrices &select_mats = preview_->selection_matrices();
	const CommonMatrixType &rot_mat = preview_->rot_mat();
	const CommonMatrixType &rot_mat2 = preview_->rot_mat2();
	const CommonMatrixType &rot_mat2_trans = preview_->rot_mat2_tr();

	QPMatrix &hessian = solver_->hessian_mat();

	int num_steps_previewed = solution.support_states_vec.back().step_number;
	int num_samples = mpc_parameters_->num_samples_horizon;

	CommonVectorType state_x(mpc_parameters_->dynamics_order), state_y(mpc_parameters_->dynamics_order);
	for (int i = 0; i < mpc_parameters_->dynamics_order; i++) {
		state_x(i) = com.x(i);
		state_y(i) = com.y(i);
	}

	// PID mode:
	// ---------
	double qx_d;
	double qy_d;
	if (mpc_parameters_->is_pid_mode) {
		const LinearDynamics &dyn = robot_->com()->dynamics_qp()[samples_left];

		double omega = sqrt(kGravity / com.z[0]);
		double kd = 1. / (1. - exp(omega * 0.06));

		// qx = - r*K*x / (BT*CT*C*A*x - BTCTCB*K*x)
		CommonMatrixType qx = -dyn.discr_ss.ss_output_mat * state_x;	//Capture point
		qx *= (1. - kd);
		qx_d = qx(0);
		CommonMatrixType bccax = dyn.discr_ss.ss_input_mat_tr * dyn.discr_ss.ss_output_mat_tr *
				dyn.discr_ss.ss_output_mat * dyn.discr_ss.ss_state_mat * state_x;
		CommonMatrixType bccbkx = dyn.discr_ss.ss_input_mat_tr * dyn.discr_ss.ss_output_mat_tr *
				dyn.discr_ss.ss_output_mat * dyn.discr_ss.ss_input_mat * (1. - kd) * dyn.discr_ss.ss_output_mat * state_x;
		qx_d /= (bccax(0) + bccbkx(0));

		// qy = - r*(1-kd)*C*y / (BT*CT*C*A*y - BTCTCB*K*y)
		CommonMatrixType qy = -(1. - kd) * dyn.discr_ss.ss_output_mat * state_y;
		qy_d = qy(0);
		CommonMatrixType bccay = dyn.discr_ss.ss_input_mat_tr * dyn.discr_ss.ss_output_mat_tr *
				dyn.discr_ss.ss_output_mat * dyn.discr_ss.ss_state_mat * state_y;
		CommonMatrixType bccbky = dyn.discr_ss.ss_input_mat_tr * dyn.discr_ss.ss_output_mat_tr *
				dyn.discr_ss.ss_output_mat * dyn.discr_ss.ss_input_mat * (1. - kd) * dyn.discr_ss.ss_output_mat * state_y;
		qy_d /= (bccay(0) + bccbky(0));

		//std::cout << "qx_d: " << qx_d << "  qy_d: " << qy_d << std::endl;

	}

	if (!solver_->do_build_cholesky()) {//TODO: This part is the most costly >50%
		if (mpc_parameters_->is_pid_mode) {
			// H = H - (1-q)*B^TC^TCB
			CommonMatrixType identity_mat = CommonMatrixType::Identity(num_samples, num_samples);

			CommonMatrixType hessian_x = const_hessian_n_mat_[samples_left];
			hessian_x -= identity_mat;
			hessian_x *= qx_d;
			hessian_x += identity_mat;
			CommonMatrixType hessian_y = const_hessian_n_mat_[samples_left];
			hessian_y -= identity_mat;
			hessian_y *= qy_d;
			hessian_y += identity_mat;
			hessian.AddTerm(hessian_x, 0, 0);
			hessian.AddTerm(hessian_y, num_samples, num_samples);
		} else {
			hessian.AddTerm(const_hessian_n_mat_[samples_left], 0, 0);
			hessian.AddTerm(const_hessian_n_mat_[samples_left], num_samples, num_samples);
		}

		CommonMatrixType hessian_mat = hessian().block(0, 0, 2 * num_samples, 2 * num_samples);
		tmp_mat_.noalias() = rot_mat2 * hessian_mat * rot_mat2_trans;//TODO: Use MTimesRT
		hessian().block(0, 0, 2 * num_samples, 2 * num_samples) = tmp_mat_;
	} else {
		// rotate the cholesky matrix
		CommonMatrixType chol = const_cholesky_[samples_left];
		RotateCholeskyMatrix(chol, rot_mat2);
		hessian.cholesky(chol);
	}

	if (num_steps_previewed > 0) {
		tmp_mat_.noalias() = const_hessian_mat_[samples_left] * select_mats.sample_step;
		hessian.AddTerm(tmp_mat_, 0, 2 * num_samples);
		hessian.AddTerm(tmp_mat_, num_samples, 2 * num_samples + num_steps_previewed);

		tmp_mat_.noalias() = select_mats.sample_step_trans * const_hessian_mat_[samples_left] * select_mats.sample_step;
		hessian.AddTerm(tmp_mat_, 2 * num_samples, 2 * num_samples);
		hessian.AddTerm(tmp_mat_, 2 * num_samples + num_steps_previewed, 2 * num_samples + num_steps_previewed);

		if (!solver_->do_build_cholesky()) {
			tmp_mat_.noalias() = select_mats.sample_step_trans * const_hessian_mat_[samples_left];
			hessian.AddTerm(tmp_mat_, 2 * num_samples, 0);
			hessian.AddTerm(tmp_mat_, 2 * num_samples + num_steps_previewed, num_samples);

			// rotate the lower left block
			// TODO(andrei): Rotation can be done before addition
			CommonMatrixType low_triang_mat = hessian().block(2 * num_samples, 0, 2 * num_steps_previewed, 2 * num_samples);
			MTimesRT(low_triang_mat, rot_mat2);
			hessian().block(2 * num_samples, 0, 2 * num_steps_previewed, 2 * num_samples) = low_triang_mat;
		}

		// rotate the upper right block
		// TODO(efficiency): Rotation can be done before addition
		CommonMatrixType upp_triang_mat = hessian().block(0, 2 * num_samples, 2*num_samples, 2*num_steps_previewed);
		RTimesM(upp_triang_mat, rot_mat2);
		hessian().block(0, 2 * num_samples, 2 * num_samples, 2 * num_steps_previewed) = upp_triang_mat;
	}

	CommonVectorType gradient_vec_x(num_samples), gradient_vec_y(num_samples), gradient_vec(2 * num_samples);//TODO: Make this member

	gradient_vec_x = state_variant_[samples_left] * state_x;
	gradient_vec_y = state_variant_[samples_left] * state_y;

	gradient_vec_x += select_variant_[samples_left] * select_mats.sample_step_cx;
	gradient_vec_y += select_variant_[samples_left] * select_mats.sample_step_cy;

	gradient_vec_x += ref_variant_vel_[samples_left] * vel_ref_->global.x;
	gradient_vec_y += ref_variant_vel_[samples_left] * vel_ref_->global.y;

	gradient_vec_x += ref_variant_pos_[samples_left] * pos_ref_->global.x;
	gradient_vec_y += ref_variant_pos_[samples_left] * pos_ref_->global.y;

	gradient_vec_x += ref_variant_cp_[samples_left] * cp_ref_->global.x;
	gradient_vec_y += ref_variant_cp_[samples_left] * cp_ref_->global.y;

	if (num_steps_previewed > 0) {
		tmp_vec_.noalias() = select_mats.sample_step_trans * gradient_vec_x;
		solver_->vector(vectorP).Add(tmp_vec_, 2 * num_samples);
		tmp_vec_.noalias() = select_mats.sample_step_trans * gradient_vec_y;
		solver_->vector(vectorP).Add(tmp_vec_, 2 * num_samples + num_steps_previewed);
	}

	if (mpc_parameters_->is_pid_mode) {
		gradient_vec_x *= qx_d;
		gradient_vec_y *= qy_d;
	}
	gradient_vec << gradient_vec_x, gradient_vec_y; //TODO: Unnecessary if rot_mat half the size
	gradient_vec = rot_mat * gradient_vec;//TODO: Use RTimesV

	solver_->vector(vectorP).Add(gradient_vec, 0);
}

void QPBuilder::BuildConstraints(const MPCSolution &solution) {
	int num_steps_previewed = solution.support_states_vec.back().step_number;

	BuildCoPConstraints(solution);
	if (num_steps_previewed>0){
		BuildFootPosInequalities(solution);
		BuildFootPosConstraints(solution);
		//BuildFootVelConstraints(solution);
	}
}

void QPBuilder::ComputeWarmStart(MPCSolution &solution) {
	//TODO: This function is not reviewed. Possibly devide this function in two parts:
	// Initialize:
	// -----------
	int num_steps = solution.support_states_vec.back().step_number;
	int num_steps_max = mpc_parameters_->num_samples_horizon;

	int nbFC = 5;// Number of foot constraints per step TODO: can be read?
	int num_samples = mpc_parameters_->num_samples_horizon;
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

void QPBuilder::BuildFootPosInequalities(const MPCSolution &solution) {
	int num_ineqs = 5;
	int num_steps = solution.support_states_vec.back().step_number;

	foot_inequalities_.Resize(num_ineqs * num_steps , num_steps);

	std::vector<SupportState>::const_iterator prev_ss_it = solution.support_states_vec.begin();
	++prev_ss_it;//Point at the first previewed instant
	for( int i = 0; i < mpc_parameters_->num_samples_horizon; ++i ){
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

void QPBuilder::BuildFootPosConstraints(const MPCSolution &solution) {
	int num_steps_previewed = solution.support_states_vec.back().step_number;
	const SelectionMatrices &select = preview_->selection_matrices();
	int num_samples = mpc_parameters_->num_samples_horizon;

	tmp_mat_.noalias() = foot_inequalities_.x_mat * select.Vf;
	solver_->constr_mat().AddTerm(tmp_mat_,  0,  2 * num_samples);

	tmp_mat_.noalias() = foot_inequalities_.y_mat * select.Vf;
	solver_->constr_mat().AddTerm(tmp_mat_,  0, 2 * num_samples + num_steps_previewed);

	solver_->vector(vectorBL).Add(foot_inequalities_.c_vec, 0);

	tmp_vec_.noalias() =  foot_inequalities_.x_mat * select.VcfX;
	tmp_vec_ += foot_inequalities_.y_mat * select.VcfY;
	solver_->vector(vectorBL).Add(tmp_vec_,  0);

	solver_->vector(vectorBU)().segment(0, tmp_vec_.size()).fill(kInf);
}

void QPBuilder::BuildFootVelConstraints(const MPCSolution &solution) {
	assert(robot_->robot_data().max_foot_vel > kEps);

	double raise_time = 0.05;//TODO: Hard coded value has to come from robot_
	double time_left = solution.support_states_vec.front().time_limit - raise_time - current_time_;
	double max_vel = robot_->robot_data().max_foot_vel;

	int num_steps_previewed = solution.support_states_vec.back().step_number;
	int x_var_pos = mpc_parameters_->num_samples_horizon;
	int y_var_pos = mpc_parameters_->num_samples_horizon + num_steps_previewed;

	const BodyState *flying_foot;
	const SupportState &current_support = solution.support_states_vec.front();
	if (current_support.foot == LEFT) {
		flying_foot = &robot_->right_foot()->state();
	} else {
		flying_foot = &robot_->left_foot()->state();
	}

	double upper_limit_x = max_vel * time_left + flying_foot->x(0);
	double upper_limit_y = max_vel * time_left + flying_foot->y(0);
	solver_->vector(vectorXU).Add(upper_limit_x, x_var_pos);
	solver_->vector(vectorXU).Add(upper_limit_y, y_var_pos);
	double lower_limit_x = -max_vel * time_left + flying_foot->x(0);
	double lower_limit_y = -max_vel * time_left + flying_foot->y(0);
	solver_->vector(vectorXL).Add(lower_limit_x, x_var_pos);
	solver_->vector(vectorXL).Add(lower_limit_y, y_var_pos);
}

void QPBuilder::BuildCoPConstraints(const MPCSolution &solution) {
	int num_samples = mpc_parameters_->num_samples_horizon;
	std::vector<SupportState>::const_iterator prev_ss_it = solution.support_states_vec.begin();

	robot_->GetConvexHull(hull_, COP_HULL, *prev_ss_it);

	int num_steps_previewed = solution.support_states_vec.back().step_number;
	int size = 2 * num_samples + 2 * num_steps_previewed;
	tmp_vec_.resize(size);
	tmp_vec2_.resize(size);

	++prev_ss_it;//Points at the first previewed instant
	for (int i = 0; i < num_samples; ++i) {
		if (prev_ss_it->state_changed) {
			robot_->GetConvexHull(hull_, COP_HULL, *prev_ss_it);
		}
		tmp_vec_(i)  = min(hull_.x_vec(0), hull_.x_vec(3));
		tmp_vec2_(i) = max(hull_.x_vec(0), hull_.x_vec(3));

		tmp_vec_(num_samples + i) = min(hull_.y_vec(0), hull_.y_vec(1));
		tmp_vec2_(num_samples + i)= max(hull_.y_vec(0), hull_.y_vec(1));
		++prev_ss_it;
	}

	tmp_vec_.segment(2 * num_samples, 2 * num_steps_previewed).fill(-kInf);
	tmp_vec2_.segment(2 * num_samples, 2 * num_steps_previewed).fill(kInf);

	int first_row = 0;
	solver_->vector(vectorXL).Add(tmp_vec_, first_row);
	solver_->vector(vectorXU).Add(tmp_vec2_, first_row);
}
