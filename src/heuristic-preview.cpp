#include <mpc-walkgen/heuristic-preview.h>

#include <mpc-walkgen/debug.h>
#include <cmath>

using namespace MPCWalkgen;
using namespace std;

HeuristicPreview::HeuristicPreview(Reference *ref, RigidBodySystem *robot, const MPCParameters *mpc_parameters, RealClock *clock)
:robot_(robot)
,mpc_parameters_(mpc_parameters)
,select_matrices_(max(mpc_parameters->num_samples_horizon_max, mpc_parameters->num_samples_state))
,rot_mat_ (CommonMatrixType::Zero(2*mpc_parameters_->num_samples_horizon_max, 2*mpc_parameters_->num_samples_horizon_max))
,rot_mat2_(CommonMatrixType::Zero(2*mpc_parameters_->num_samples_horizon_max, 2*mpc_parameters_->num_samples_horizon_max))
,rot_mat2_tr_(CommonMatrixType::Zero(2*mpc_parameters_->num_samples_horizon_max, 2*mpc_parameters_->num_samples_horizon_max))
,clock_(clock) {
	support_fsm_ = new StateFSM(ref, mpc_parameters);
}

HeuristicPreview::~HeuristicPreview() {
	delete support_fsm_;
}

void HeuristicPreview::PreviewSamplingTimes(double current_time,
		double first_fine_period, double first_coarse_period, MPCSolution &solution) {

	if (mpc_parameters_->mapping == ZERO_MAP) {
		solution.sampling_times_vec.resize(mpc_parameters_->num_samples_horizon_max + 1, 0.);//TODO
		std::fill(solution.sampling_times_vec.begin(), solution.sampling_times_vec.end(), 0.);
		solution.sampling_times_vec.at(0) = current_time;
		if (mpc_parameters_->formulation == STANDARD) {
			solution.sampling_times_vec[1] = solution.sampling_times_vec[0] + first_coarse_period;

			for (int sample = 2; sample < mpc_parameters_->num_samples_horizon_max + 1; sample++) {
				solution.sampling_times_vec[sample] += solution.sampling_times_vec[sample - 1] + mpc_parameters_->period_qpsample;
			}
		} else {
			int sample = 1;
			// First grid
			for (; sample <= mpc_parameters_->num_samples_first_fine_period; sample++) {
				solution.sampling_times_vec.at(sample) = solution.sampling_times_vec.at(sample - 1) + mpc_parameters_->period_recomputation;
			}
			// Second grid
			solution.sampling_times_vec.at(sample) = solution.sampling_times_vec.at(0) + first_fine_period + mpc_parameters_->period_inter_samples;
			sample++;
			for (; sample < mpc_parameters_->num_samples_first_coarse_period + mpc_parameters_->num_samples_first_fine_period; sample++) {
				solution.sampling_times_vec.at(sample) = solution.sampling_times_vec.at(sample - 1) + mpc_parameters_->period_inter_samples;
			}
			// Third grid (fixed)
			solution.sampling_times_vec.at(sample) = solution.sampling_times_vec.at(0) + first_coarse_period + mpc_parameters_->period_qpsample;
			sample++;
			for (; sample < mpc_parameters_->num_samples_horizon_max; sample++) { //TODO: only num_samples_horizon
				solution.sampling_times_vec.at(sample) = solution.sampling_times_vec.at(sample - 1) + mpc_parameters_->period_qpsample;
			}
			// Last sample (sliding)
			solution.sampling_times_vec.at(sample) = solution.sampling_times_vec.at(sample - 1) + mpc_parameters_->period_qpsample - first_coarse_period;
			if (mpc_parameters_->period_qpsample - first_coarse_period < kEps && mpc_parameters_->formulation != STANDARD) {
				solution.sampling_times_vec.pop_back();
			}
		}
	} else if (mpc_parameters_->mapping == CONST_MAP) {
		solution.sampling_times_vec.resize(mpc_parameters_->num_samples_state + 1, 0.); //TODO: Hard coded
		std::vector<double>::iterator st_it = solution.sampling_times_vec.begin();
		std::vector<double>::iterator stm1_it = solution.sampling_times_vec.begin();
		*st_it = current_time;
		st_it++;// First previewed sample
		while (st_it != solution.sampling_times_vec.end()) {
			*st_it = *stm1_it + mpc_parameters_->period_actsample;
			st_it++;
			stm1_it++;
		}
	}
}

void HeuristicPreview::PreviewSupportStates(double first_sample_period, MPCSolution &solution) {
	// SET CURRENT SUPPORT STATE:
	// --------------------------
	const BodyState *foot;
	SupportState &current_support = robot_->current_support();
	double current_time = solution.sampling_times_vec[0];
	solution.sampling_times_vec[0] += mpc_parameters_->period_recomputation;// Quickfix
	support_fsm_->SetSupportState(current_time, 0, solution.sampling_times_vec, current_support);
	solution.sampling_times_vec[0] = current_time;// Quickfix
	if (current_support.state_changed) {
		if (current_support.foot == LEFT) {
			foot = &robot_->left_foot()->state();
		} else {
			foot = &robot_->right_foot()->state();
		}
		current_support.x = foot->x(0);
		current_support.y = foot->y(0);
		current_support.yaw = foot->yaw(0);
		current_support.start_time = solution.sampling_times_vec[0];
	}
	if (solution.sampling_times_vec[0] > current_support.start_time + mpc_parameters_->period_trans_ds() - kEps) {
		current_support.transitional_ds = false;
	}
	solution.support_states_vec.push_back(current_support);

	// PREVIEW SUPPORT STATES:
	// -----------------------
	// initialize the previewed support state before previewing
	SupportState previewed_support = current_support;//TODO: Replace =operator by CopyFrom or give to constructor
	previewed_support.step_number = 0;
	int ns_st = static_cast<int>(solution.sampling_times_vec.size());
	for (int sample = 1; sample < ns_st; sample++) {
		support_fsm_->SetSupportState(current_time, sample, solution.sampling_times_vec, previewed_support);
		// special treatment for the first instant of transitionalDS
		if (previewed_support.step_number > 0) {
			previewed_support.x = 0.0;
			previewed_support.y = 0.0;
		}
		if (previewed_support.transitional_ds) {
			// robot is already in ds phase
			if (previewed_support.foot == LEFT) {
				foot = &robot_->left_foot()->state();
			} else {
				foot = &robot_->right_foot()->state();
			}
			previewed_support.x = foot->x(0);
			previewed_support.y = foot->y(0);
			previewed_support.yaw = foot->yaw(0);
		}
		/*
		if (sample == 1) {
			previewed_support.previous_sampling_period = first_sample_period;
		} else {
			previewed_support.previous_sampling_period = mpc_parameters_->period_qpsample;
		}
		 */
		solution.support_states_vec.push_back(previewed_support);
	}

	BuildSelectionMatrices(solution);
}

void HeuristicPreview::BuildRotationMatrix(MPCSolution &solution){//TODO: Move to qp-builder
	int num_samples = mpc_parameters_->num_samples_contr;

	for (int i=0; i<num_samples; ++i) {//TODO:(performance)
		double cos_yaw = cos(solution.support_states_vec[i+1].yaw);
		double sin_yaw = sin(solution.support_states_vec[i+1].yaw);
		rot_mat_(i  ,i  ) =  cos_yaw;
		rot_mat_(i+num_samples,i  ) = -sin_yaw;
		rot_mat_(i  ,i+num_samples) =  sin_yaw;
		rot_mat_(i+num_samples,i+num_samples) =  cos_yaw;

		rot_mat2_(2*i  , 2*i  ) =  cos_yaw;
		rot_mat2_(2*i+1, 2*i  ) = -sin_yaw;
		rot_mat2_(2*i  , 2*i+1) =  sin_yaw;
		rot_mat2_(2*i+1, 2*i+1) =  cos_yaw;

		rot_mat2_tr_(2*i  , 2*i  ) = cos_yaw;
		rot_mat2_tr_(2*i+1, 2*i  ) = sin_yaw;
		rot_mat2_tr_(2*i  , 2*i+1) = -sin_yaw;
		rot_mat2_tr_(2*i+1, 2*i+1) = cos_yaw;

	}

}

//
// Private methods:
//
void HeuristicPreview::BuildSelectionMatrices(MPCSolution &solution) {//Move to qp-builder
	const BodyState *left_foot_p = &robot_->left_foot()->state();
	const BodyState *right_foot_p = &robot_->right_foot()->state();

	int num_steps_previewed = solution.support_states_vec.back().step_number;
	int num_samples = mpc_parameters_->num_samples_state;
	int num_rows = num_samples;
	if (select_matrices_.sample_step.cols() != num_steps_previewed || select_matrices_.sample_step.rows() != num_samples) {
		select_matrices_.sample_step.		resize(num_rows, num_steps_previewed);
		select_matrices_.sample_step_cx.	resize(num_rows);
		select_matrices_.sample_step_cy.	resize(num_rows);
		select_matrices_.sample_step_trans.	resize(num_steps_previewed, num_rows);
		select_matrices_.Vf.				resize(num_steps_previewed, num_steps_previewed);
		select_matrices_.VcfX.				resize(num_steps_previewed);
		select_matrices_.VcfY.				resize(num_steps_previewed);
	}
	select_matrices_.SetZero();

	std::vector<SupportState>::iterator supp_state_it = solution.support_states_vec.begin();//points at the cur. sup. st.
	++supp_state_it;
	for (int i = 0; i < num_samples; i++) {
		if (supp_state_it->step_number > 0) {
			select_matrices_.sample_step(i, supp_state_it->step_number - 1) = select_matrices_.sample_step_trans(supp_state_it->step_number-1, i) = 1.0;
			if (supp_state_it->step_number == 1 && supp_state_it->state_changed && supp_state_it->phase == SS) {
				--supp_state_it;
				select_matrices_.VcfX(0) = supp_state_it->x;
				select_matrices_.VcfY(0) = supp_state_it->y;
				++supp_state_it;

				select_matrices_.Vf(0,0) = 1.0;
			} else if(supp_state_it->step_number > 1) {
				select_matrices_.Vf(supp_state_it->step_number - 1, supp_state_it->step_number - 2) = -1.0;
				select_matrices_.Vf(supp_state_it->step_number - 1, supp_state_it->step_number - 1) = 1.0;
			}
		} else {
			select_matrices_.sample_step_cx(i) = supp_state_it->x;
			select_matrices_.sample_step_cy(i) = supp_state_it->y;
		}
		++supp_state_it;
	}
	// Copy last row generated above to the following (last) row (concerning the capture point position)
	///select_matrices_.sample_step_cx(num_samples) = select_matrices_.sample_step_cx(num_samples - 1);
	///select_matrices_.sample_step_cy(num_samples) = select_matrices_.sample_step_cy(num_samples - 1);
	///int num_cols = select_matrices_.sample_step.cols();
	///select_matrices_.sample_step.block(num_samples, 0, 1, num_cols) = select_matrices_.sample_step.block(num_samples - 1, 0, 1, num_cols);
	///select_matrices_.sample_step_trans.block(0, num_samples, num_cols, 1) = select_matrices_.sample_step_trans.block(0, num_samples - 1, num_cols, 1);
}
