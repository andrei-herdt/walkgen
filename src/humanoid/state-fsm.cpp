#include "state-fsm.h"

using namespace MPCWalkgen;


StateFSM::StateFSM(Reference * velRef, const MPCData * data_mpc)
:velRef_(velRef)
,data_mpc_(data_mpc)
{}

StateFSM::~StateFSM(){}


void StateFSM::setSupportState(int sample, const std::vector<double> &samplingTimes_vec, SupportState &support) {

  support.state_changed = false;
  support.nbInstants++;

  bool ReferenceGiven = false;
  if (fabs(velRef_->local.x(0)) > EPSILON || fabs(velRef_->local.y(0)) > EPSILON || fabs(velRef_->local.yaw(0)) > EPSILON) {
    ReferenceGiven = true;
  }

  // Update time limit for double support phase
  if (ReferenceGiven && support.phase == DS && 
      support.time_limit > samplingTimes_vec[data_mpc_->nbqpsamples_dsss] - EPSILON) {
      support.time_limit = samplingTimes_vec[data_mpc_->nbqpsamples_dsss];
      support.nbStepsLeft = data_mpc_->nbsteps_ssds;
  }

  //FSM logic
  if (samplingTimes_vec[sample] >= support.time_limit - EPSILON) {
    //SS->DS
    if (support.phase == SS && !ReferenceGiven && support.nbStepsLeft == 0){
      support.phase 			  = DS;
      support.time_limit 		= samplingTimes_vec[sample] + data_mpc_->period_ds;
      support.state_changed 	= true;
      support.nbInstants 		= 0;
      //DS->SS
    } else if (((support.phase == DS) && ReferenceGiven) || ((support.phase == DS) && (support.nbStepsLeft > 0))){
      support.phase         = SS;
      support.time_limit 		= samplingTimes_vec[sample] + data_mpc_->nbqpsamples_step * data_mpc_->period_qpsample;
      support.nbStepsLeft 	= data_mpc_->nbsteps_ssds;
      support.state_changed 	= true;
      support.nbInstants 		= 0;
      //SS->SS
    } else if ((support.phase == SS && support.nbStepsLeft > 0) || (support.nbStepsLeft == 0 && ReferenceGiven)){
      if (support.foot == LEFT){
        support.foot = RIGHT;
      } else {
        support.foot = LEFT;
      }
      support.state_changed 	= true;
      support.nbInstants 		= 0;
      support.time_limit 		= samplingTimes_vec[sample] + data_mpc_->nbqpsamples_step * data_mpc_->period_qpsample;
      if (sample != 1) {//Flying foot is not down
        ++support.stepNumber;
      }
      if (!ReferenceGiven) {
        support.nbStepsLeft = support.nbStepsLeft-1;
      }
      if (ReferenceGiven) {
        support.nbStepsLeft = data_mpc_->nbsteps_ssds;
      }
    }
  }
}
