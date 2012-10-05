#include <mpc-walkgen/state-fsm.h>

using namespace MPCWalkgen;

StateFSM::StateFSM(Reference *ref, const MPCParameters *mpc_parameters)
:vel_ref_(ref)
,mpc_parameters_(mpc_parameters)
{}

StateFSM::~StateFSM(){}


void StateFSM::SetSupportState(int sample, const std::vector<double> &sampling_times_vec, SupportState &support) {

  support.state_changed = false;
  support.num_instants++;

  bool ReferenceGiven = false;
  if (fabs(vel_ref_->local.x(0)) > kEps || fabs(vel_ref_->local.y(0)) > kEps || fabs(vel_ref_->local.yaw(0)) > kEps) {
    ReferenceGiven = true;
  }

  // Update time limit for double support phase
  if (ReferenceGiven && support.phase == DS && 
      support.time_limit > sampling_times_vec[mpc_parameters_->num_samples_dsss] - kEps) {
      support.time_limit = sampling_times_vec[mpc_parameters_->num_samples_dsss];
      support.num_steps_left = mpc_parameters_->num_steps_ssds;
  }

  //FSM logic
  if (sampling_times_vec[sample] >= support.time_limit - kEps) {
    //SS->DS
    if (support.phase == SS && !ReferenceGiven && support.num_steps_left == 0){
      support.phase 			  = DS;
      support.time_limit 		= sampling_times_vec[sample] + mpc_parameters_->period_ds;
      support.state_changed 	= true;
      support.num_instants 		= 0;
      //DS->SS
    } else if (((support.phase == DS) && ReferenceGiven) || ((support.phase == DS) && (support.num_steps_left > 0))){
      support.phase         = SS;
      support.time_limit 		= sampling_times_vec[sample] + mpc_parameters_->num_samples_step * mpc_parameters_->period_qpsample;
      support.num_steps_left 	= mpc_parameters_->num_steps_ssds;
      support.state_changed 	= true;
      support.num_instants 		= 0;
      //SS->SS
    } else if ((support.phase == SS && support.num_steps_left > 0) || (support.num_steps_left == 0 && ReferenceGiven)){
      if (support.foot == LEFT){
        support.foot = RIGHT;
      } else {
        support.foot = LEFT;
      }
      support.state_changed 	= true;
      support.num_instants 		= 0;
      support.time_limit 		= sampling_times_vec[sample] + mpc_parameters_->num_samples_step * mpc_parameters_->period_qpsample;
      if (sample != 1) {//Flying foot is not down
        ++support.step_number;
      }
      if (!ReferenceGiven) {
        support.num_steps_left = support.num_steps_left-1;
      }
      if (ReferenceGiven) {
        support.num_steps_left = mpc_parameters_->num_steps_ssds;
      }
    }
  }
}
