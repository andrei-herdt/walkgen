#include "state-fsm.h"

using namespace MPCWalkgen;


StateFSM::StateFSM(Reference * velRef, const MPCData * generalData)
:velRef_(velRef)
,generalData_(generalData)
{}

StateFSM::~StateFSM(){}


void StateFSM::setSupportState(int sample, const std::vector<double> &samplingTimes_vec, SupportState & support) {

  support.stateChanged = false;
  support.nbInstants++;

  bool ReferenceGiven = false;
  if (fabs(velRef_->local.x(0)) > EPSILON || fabs(velRef_->local.y(0)) > EPSILON || fabs(velRef_->local.yaw(0)) > EPSILON) {
    ReferenceGiven = true;
  }

  // Update time limit for double support phase
  if (ReferenceGiven && support.phase == DS && 
      support.timeLimit > samplingTimes_vec[generalData_->nbqpsamples_dsss] - EPSILON) {
      support.timeLimit = samplingTimes_vec[generalData_->nbqpsamples_dsss];
      support.nbStepsLeft = generalData_->nbsteps_ssds;
  }

  //FSM logic
  if (samplingTimes_vec[sample] >= support.timeLimit - EPSILON) {
    //SS->DS
    if (support.phase == SS && !ReferenceGiven && support.nbStepsLeft == 0){
      support.phase 			  = DS;
      support.timeLimit 		= samplingTimes_vec[sample] + generalData_->period_ds;
      support.stateChanged 	= true;
      support.nbInstants 		= 0;
      //DS->SS
    } else if (((support.phase == DS) && ReferenceGiven) || ((support.phase == DS) && (support.nbStepsLeft > 0))){
      support.phase         = SS;
      support.timeLimit 		= samplingTimes_vec[sample] + generalData_->nbqpsamples_step * generalData_->period_qpsample;
      support.nbStepsLeft 	= generalData_->nbsteps_ssds;
      support.stateChanged 	= true;
      support.nbInstants 		= 0;
      //SS->SS
    } else if ((support.phase == SS && support.nbStepsLeft > 0) || (support.nbStepsLeft == 0 && ReferenceGiven)){
      if (support.foot == LEFT){
        support.foot = RIGHT;
      } else {
        support.foot = LEFT;
      }
      support.stateChanged 	= true;
      support.nbInstants 		= 0;
      support.timeLimit 		= samplingTimes_vec[sample] + generalData_->nbqpsamples_step * generalData_->period_qpsample;
      if (sample != 1) {//Flying foot is not down
        ++support.stepNumber;
      }
      if (!ReferenceGiven) {
        support.nbStepsLeft = support.nbStepsLeft-1;
      }
      if (ReferenceGiven) {
        support.nbStepsLeft = generalData_->nbsteps_ssds;
      }
    }
  }
}
