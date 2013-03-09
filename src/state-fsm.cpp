#include <mpc-walkgen/state-fsm.h>

#include <mpc-walkgen/debug.h>

using namespace MPCWalkgen;

StateFSM::StateFSM(Reference *ref, const MPCParameters *mpc_parameters)
:vel_ref_(ref)
,mpc_parameters_(mpc_parameters)
{}

StateFSM::~StateFSM(){}


void StateFSM::SetSupportState(int sample, const std::vector<double> &sampling_times_vec, SupportState &support) {

	support.state_changed = false;
	support.num_instants++;

	bool is_reference_given = false;
	if (fabs(vel_ref_->local.x(0)) > kEps || fabs(vel_ref_->local.y(0)) > kEps || fabs(vel_ref_->local.yaw(0)) > kEps) {
		is_reference_given = true;
	}

	// Update time limit for double support phase
	if (is_reference_given && support.phase == DS &&
			support.time_limit > sampling_times_vec[mpc_parameters_->num_samples_dsss] + kEps) {
		support.time_limit = sampling_times_vec[mpc_parameters_->num_samples_dsss];
		support.num_steps_left = mpc_parameters_->num_steps_ssds;
	}

	//FSM logic
	if (sampling_times_vec[sample] >= support.time_limit + mpc_parameters_->period_recomputation / 2.) {
		//SS->DS
		if (support.phase == SS && !is_reference_given && support.num_steps_left == 0){
			support.phase 			= DS;
			support.start_time 		= support.time_limit;
			support.time_limit 		+= mpc_parameters_->period_ds;
			support.state_changed 	= true;
			support.num_instants 		= 0;
			//DS->SS
		} else if ((support.phase == DS && is_reference_given) || (support.phase == DS && (support.num_steps_left > 0))){
			support.phase         	= SS;
			//support.transitional_ds 	= true;
			support.start_time 		= support.time_limit;
			support.time_limit 		+= mpc_parameters_->period_trans_ds() + mpc_parameters_->period_ss;
			support.num_steps_left 	= mpc_parameters_->num_steps_ssds;
			support.state_changed 	= true;
			support.num_instants 		= 0;
			//SS->SS
		} else if ((support.phase == SS && support.num_steps_left > 0) || (support.num_steps_left == 0 && is_reference_given)){
			if (support.foot == LEFT){
				support.foot = RIGHT;
			} else {
				support.foot = LEFT;
			}
			support.state_changed 	= true;
			support.num_instants 		= 0;
			support.transitional_ds 	= true;
			support.start_time 		= support.time_limit;
			support.time_limit 		+= mpc_parameters_->period_trans_ds() + mpc_parameters_->period_ss;
			//if (sample != 1) {//Flying foot is not down
			++support.step_number;
			//}
			if (!is_reference_given) {
				support.num_steps_left = support.num_steps_left-1;
			}
			if (is_reference_given) {
				support.num_steps_left = mpc_parameters_->num_steps_ssds;
			}
		}
	}

	// Transitional double support phase
	if (sampling_times_vec[sample] > support.start_time + mpc_parameters_->period_trans_ds() + kEps) {
		support.transitional_ds = false;
	}

}
