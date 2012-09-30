#include <mpc-walkgen/heuristic-preview.h>

#include <cmath>

using namespace MPCWalkgen;

HeuristicPreview::HeuristicPreview(Reference *ref, RigidBodySystem *robot,
		const MPCParameters *mpc_parameters, RealClock *clock)
:robot_(robot)
,mpc_parameters_(mpc_parameters)
,select_matrices_(mpc_parameters->num_samples_horizon)
,rot_mat_ (CommonMatrixType::Zero(2*mpc_parameters_->num_samples_horizon, 2*mpc_parameters_->num_samples_horizon))
,rot_mat2_(CommonMatrixType::Zero(2*mpc_parameters_->num_samples_horizon, 2*mpc_parameters_->num_samples_horizon))
,rot_mat2_tr_(CommonMatrixType::Zero(2*mpc_parameters_->num_samples_horizon, 2*mpc_parameters_->num_samples_horizon))
,clock_(clock) {
	support_fsm_ = new StateFSM(ref, mpc_parameters);
}

HeuristicPreview::~HeuristicPreview() {
	delete support_fsm_;
}

void HeuristicPreview::PreviewSamplingTimes(double current_time,
		double firstSamplingPeriod, MPCSolution &solution) {

	solution.sampling_times_vec.resize(mpc_parameters_->num_samples_horizon + 1, 0);
	std::fill(solution.sampling_times_vec.begin(), solution.sampling_times_vec.end(), 0);
	// As for now, only the first sampling period varies
	solution.sampling_times_vec[0] = current_time;
	solution.sampling_times_vec[1] = solution.sampling_times_vec[0] + firstSamplingPeriod;// mpc_parameters_->QPSamplingPeriod;////// //
	for (int sample = 2; sample < mpc_parameters_->num_samples_horizon + 1; sample++) {
		solution.sampling_times_vec[sample] += solution.sampling_times_vec[sample - 1] +
				mpc_parameters_->period_qpsample;
	}

}

void HeuristicPreview::PreviewSupportStates(double first_sample_period, MPCSolution &solution)
{
	const BodyState *foot;
	SupportState &current_support = robot_->current_support();

	// SET CURRENT SUPPORT STATE:
	// --------------------------
	support_fsm_->setSupportState(0, solution.sampling_times_vec, current_support);
	current_support.transitional_ds = false;
	if (current_support.state_changed) {
		if (current_support.foot == LEFT) {
			foot = &robot_->body(LEFT_FOOT)->state();
		} else {
			foot = &robot_->body(RIGHT_FOOT)->state();
		}
		current_support.x = foot->x(0);
		current_support.y = foot->y(0);
		current_support.yaw = foot->yaw(0);
		current_support.start_time = solution.sampling_times_vec[0];
	}
	solution.support_states_vec.push_back(current_support);

	// PREVIEW SUPPORT STATES:
	// -----------------------
	// initialize the previewed support state before previewing
	SupportState previewed_support = current_support;//TODO: Replace =operator by CopyFrom or give to constructor
	previewed_support.step_number = 0;
	for (int sample = 1; sample <= mpc_parameters_->num_samples_horizon; sample++) {
		support_fsm_->setSupportState(sample, solution.sampling_times_vec, previewed_support);
		// special treatment for the first instant of transitionalDS
		previewed_support.transitional_ds = false;
		if (previewed_support.state_changed) {
			if (sample == 1) {// robot is already in ds phase
				if (previewed_support.foot == LEFT) {
					foot = &robot_->body(LEFT_FOOT)->state();
				} else {
					foot = &robot_->body(RIGHT_FOOT)->state();
				}
				previewed_support.x = foot->x(0);
				previewed_support.y = foot->y(0);
				previewed_support.yaw = foot->yaw(0);
				previewed_support.start_time = solution.sampling_times_vec[sample];
				if (current_support.phase == SS && previewed_support.phase == SS) {
					previewed_support.transitional_ds = true;
				}
			}
			if (previewed_support.step_number > 0) {
				previewed_support.x = 0.0;
				previewed_support.y = 0.0;
			}
		}
		if (sample == 1) {
			previewed_support.previousSamplingPeriod = first_sample_period;
			previewed_support.sampleWeight = 1;
		} else {
			previewed_support.previousSamplingPeriod = mpc_parameters_->period_qpsample;
			previewed_support.sampleWeight = 1;
		}
		solution.support_states_vec.push_back(previewed_support);
	}

	BuildSelectionMatrices(solution);
}

void HeuristicPreview::BuildRotationMatrix(MPCSolution &solution){
	int num_samples = mpc_parameters_->num_samples_horizon;

	for (int i=0; i<num_samples; ++i) {//TODO:(performance) Vecotrize this?
		double cosYaw = cos(solution.support_states_vec[i+1].yaw);
		double sinYaw = sin(solution.support_states_vec[i+1].yaw);
		rot_mat_(i  ,i  ) =  cosYaw;
		rot_mat_(i+num_samples,i  ) = -sinYaw;
		rot_mat_(i  ,i+num_samples) =  sinYaw;
		rot_mat_(i+num_samples,i+num_samples) =  cosYaw;

		rot_mat2_(2*i  , 2*i  ) =  cosYaw;
		rot_mat2_(2*i+1, 2*i  ) = -sinYaw;
		rot_mat2_(2*i  , 2*i+1) =  sinYaw;
		rot_mat2_(2*i+1, 2*i+1) =  cosYaw;

		rot_mat2_tr_(2*i  , 2*i  ) = cosYaw;
		rot_mat2_tr_(2*i+1, 2*i  ) = sinYaw;
		rot_mat2_tr_(2*i  , 2*i+1) = -sinYaw;
		rot_mat2_tr_(2*i+1, 2*i+1) = cosYaw;

	}

}

//
// Private methods:
//
void HeuristicPreview::BuildSelectionMatrices(MPCSolution &solution) {
	assert(select_matrices_.sample_step.rows() == mpc_parameters_->num_samples_horizon);

	const BodyState *left_foot_p = &robot_->body(LEFT_FOOT)->state();
	const BodyState *right_foot_p = &robot_->body(RIGHT_FOOT)->state();

	int num_steps_previewed = solution.support_states_vec.back().step_number;
	int num_samples = mpc_parameters_->num_samples_horizon;

	if (select_matrices_.sample_step.cols() != num_steps_previewed){
		select_matrices_.sample_step.resize(num_samples, num_steps_previewed);
		select_matrices_.sample_step_trans.resize(num_steps_previewed, num_samples);
		select_matrices_.Vf.resize(num_steps_previewed, num_steps_previewed);
		select_matrices_.VcfX.resize(num_steps_previewed);
		select_matrices_.VcfY.resize(num_steps_previewed);
		select_matrices_.sample_mstep.resize(num_samples, num_steps_previewed);
		select_matrices_.sample_mstep_trans.resize(num_steps_previewed, num_samples);
	}
	select_matrices_.SetZero();

	std::vector<SupportState>::iterator supp_state_it = solution.support_states_vec.begin();//points at the cur. sup. st.
	++supp_state_it;
	for (int i = 0; i < num_samples; i++) {
		if (supp_state_it->step_number > 0) {
			select_matrices_.sample_step(i, supp_state_it->step_number - 1) = select_matrices_.sample_step_trans(supp_state_it->step_number-1, i) = 1.0;
			select_matrices_.sample_mstep(i, supp_state_it->step_number - 1) = select_matrices_.sample_mstep_trans(supp_state_it->step_number-1, i) = 1.0;
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
			select_matrices_.sample_mstep_cx(i) = supp_state_it->x;
			select_matrices_.sample_mstep_cy(i) = supp_state_it->y;
			if (supp_state_it->phase == DS) {
				select_matrices_.sample_mstep_cx(i) = left_foot_p->x(0) / 2. + right_foot_p->x(0) / 2.;
				select_matrices_.sample_mstep_cy(i) = left_foot_p->y(0) / 2. + right_foot_p->y(0) / 2.;
			}
		}
		++supp_state_it;
	}

}
