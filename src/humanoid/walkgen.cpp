#include "walkgen.h"

#include "orientations-preview.h"
#include "../common/qp-solver.h"
#include "qp-generator.h"
#include "qp-preview.h"
#include "rigid-body-system.h"
#include "../common/interpolation.h"
#include "../common/mpc-debug.h"

#include <iostream>
#include <windows.h> 
#include <Eigen/Dense>
#include <stdio.h>

using namespace MPCWalkgen;

using namespace Eigen;


MPCWalkgen::WalkgenAbstract* MPCWalkgen::createWalkgen(MPCWalkgen::QPSolverType solvertype) {
  MPCWalkgen::WalkgenAbstract* zmpVra = new MPCWalkgen::Walkgen(solvertype);
  return zmpVra;
}




// Implementation of the private interface
Walkgen::Walkgen(::MPCWalkgen::QPSolverType solvertype)
: WalkgenAbstract()
,generalData_()
,solver_(0x0)
,generator_(0x0)
,preview_(0x0)
,interpolation_(0x0)
,robot_(0x0)
,orientPrw_(0x0)
,output_()
,output_index_(0)
,solution_()
,velRef_()
,newCurrentSupport_()
,isNewCurrentSupport_(false)
,first_sample_time_(0)
,next_computation_(0)
,next_act_sample_(0)
,currentTime_(0)
,currentRealTime_(0)
{
  int nb_steps_max = 2;
  solver_ = createQPSolver(solvertype,
    2 * (generalData_.nbsamples_qp + nb_steps_max), 
    5 * nb_steps_max
    );

  orientPrw_ = new OrientationsPreview();

  interpolation_ = new Interpolation();

  robot_ = new RigidBodySystem(&generalData_);

  preview_ = new QPPreview(&velRef_, robot_, &generalData_);

  generator_= new QPGenerator(preview_, solver_, &velRef_, &ponderation_, robot_, &generalData_);

}


Walkgen::~Walkgen(){
  if (orientPrw_ != 0x0)
    delete orientPrw_;

  if (solver_ != 0x0)
    delete solver_;

  if (generator_ != 0x0)
    delete generator_;

  if (preview_ != 0x0)
    delete preview_;

  if (robot_ != 0x0)
    delete robot_;

  if (interpolation_ != 0x0)
    delete interpolation_;

}

void Walkgen::init(const RobotData &robotData, const MPCData &mpcData) {

  generalData_ = mpcData;
  robotData_ = robotData;

  init();

}

void Walkgen::init() {
  robot_->Init(robotData_, interpolation_);

  //Check if sampling periods are defined correctly
  assert(generalData_.period_actsample > 0);
  assert(generalData_.period_mpcsample >= generalData_.period_actsample);
  assert(generalData_.period_qpsample >= generalData_.period_mpcsample);
  assert(generalData_.nbqpsamples_step <= generalData_.nbsamples_qp);
  assert(generalData_.nbqpsamples_dsss <= generalData_.nbsamples_qp);

  // Redistribute the X,Y vectors of variables inside the optimization problems
  VectorXi order(solver_->nbvar_max());
  for (int i = 0; i < generalData_.nbsamples_qp; ++i) {// 0,2,4,1,3,5 (CoM)
    order(i) = 2*i;
    order(i + generalData_.nbsamples_qp) = 2*i+1;
  }

  for (int i = 2*generalData_.nbsamples_qp; i < solver_->nbvar_max(); ++i) {// 6,7,8 (Feet)
    order(i) = i;
  }

  solver_->varOrder(order);

  orientPrw_->init(generalData_, robotData_);

  robot_->computeDynamics();

  generator_->precomputeObjective();

  BodyState state;
  state.x[0] = robotData_.leftFootPos[0];
  state.y[0] = robotData_.leftFootPos[1];
  robot_->body(LEFT_FOOT)->state(state);

  state.x[0] = robotData_.rightFootPos[0];
  state.y[0] = robotData_.rightFootPos[1];
  robot_->body(RIGHT_FOOT)->state(state);

  state.x[0] = robotData_.com(0);//TODO: Add macros for x,y,z
  state.y[0] = robotData_.com(1);
  state.z[0] = robotData_.com(2);

  robot_->body(COM)->state(state);

  ponderation_.activePonderation = 0;

  velRef_.resize(generalData_.nbsamples_qp);
  newVelRef_.resize(generalData_.nbsamples_qp);

  BuildProblem();

  solver_->Init();
}

const MPCSolution & Walkgen::online(){
  currentRealTime_ += generalData_.period_mpcsample;
  return online(currentRealTime_);
}

const MPCSolution & Walkgen::online(double time){
  currentTime_ = time;

  if (time  > next_computation_ - EPSILON) {
    next_computation_ += generalData_.period_mpcsample;
    if (time > next_computation_ - EPSILON) {   
      ResetCounters(time);
    }
    if(time  > first_sample_time_ - EPSILON){
      first_sample_time_ += generalData_.period_qpsample;
      if (time > first_sample_time_ - EPSILON) {
        ResetCounters(time);
      }
    }
    ResetOutputIndex();

    BuildProblem();

    solver_->solve(solution_.qpSolution, solution_.constraints,
      solution_.initialSolution, solution_.initialConstraints,
      generalData_.warmstart);

    GenerateTrajectories();
  }

  if (time > next_act_sample_ - EPSILON) {
    next_act_sample_ += generalData_.period_actsample;

    IncrementOutputIndex();
    UpdateOutput();
  }

  return solution_;
}

void Walkgen::SetCounters(double time) {

}
void Walkgen::ResetCounters(double time) {
  first_sample_time_ = time + generalData_.period_qpsample;
  next_computation_ = time + generalData_.period_mpcsample;
  next_act_sample_ = time + generalData_.period_actsample;
}

void Walkgen::BuildProblem() {
  // UPDATE INTERNAL DATA:
  // ---------------------
  solver_->reset();
  solution_.reset();
  velRef_ = newVelRef_;
  if (isNewCurrentSupport_){
    robot_->currentSupport(newCurrentSupport_);
    isNewCurrentSupport_ = false;
  }

  if (robot_->currentSupport().phase == SS && robot_->currentSupport().nbStepsLeft == 0) {
    velRef_.local.x.fill(0);
    velRef_.local.y.fill(0);
    velRef_.local.yaw.fill(0);//TODO: Bullshit
  }
  if (velRef_.local.yaw(0) == 0 && velRef_.local.x(0) == 0 && velRef_.local.y(0) == 0) {
    ponderation_.activePonderation = 1;
  } else {
    ponderation_.activePonderation = 0;
  }

  double firstSamplingPeriod = first_sample_time_ - currentTime_;
  robot_->setSelectionNumber(firstSamplingPeriod);

  // PREVIEW:
  // --------
  preview_->previewSamplingTimes(currentTime_, firstSamplingPeriod, solution_);

  preview_->previewSupportStates(firstSamplingPeriod, solution_);

  orientPrw_->preview_orientations( currentTime_, velRef_, 
    generalData_.nbqpsamples_step * generalData_.period_qpsample,
    robot_->body(LEFT_FOOT)->state(), robot_->body(RIGHT_FOOT)->state(),
    solution_ );
  preview_->computeRotationMatrix(solution_);

  generator_->computeReferenceVector(solution_);

  // BUILD:
  // ------
  generator_->BuildProblem(solution_);
}

void Walkgen::GenerateTrajectories() {
  generator_->convertCopToJerk(solution_);

  robot_->interpolateBodies(solution_, currentTime_, velRef_);

  robot_->updateBodyState(solution_);

  orientPrw_->interpolate_trunk_orientation(robot_);

  ResetOutputIndex();
  UpdateOutput();
}


void Walkgen::ResetOutputIndex() {
  output_index_ = 0;
}

void Walkgen::IncrementOutputIndex() {
  if (output_index_ < generalData_.nbSamplesControl() - 1) {
    output_index_++;
  }
}

void Walkgen::UpdateOutput() {
  output_.com.x = solution_.state_vec[0].CoMTrajX_[output_index_];
  output_.com.y = solution_.state_vec[0].CoMTrajY_[output_index_];
  output_.com.z = robot_->body(COM)->state().z(0);
  output_.com.dx = solution_.state_vec[1].CoMTrajX_[output_index_];
  output_.com.dy = solution_.state_vec[1].CoMTrajY_[output_index_];
  output_.com.dz = robot_->body(COM)->state().z(1);
  output_.com.ddx = solution_.state_vec[2].CoMTrajX_[output_index_];
  output_.com.ddy = solution_.state_vec[2].CoMTrajY_[output_index_];
  output_.com.ddz = robot_->body(COM)->state().z(2);

  output_.cop.x = solution_.CoPTrajX[output_index_];
  output_.cop.y = solution_.CoPTrajY[output_index_];

  output_.left_foot.x = solution_.state_vec[0].leftFootTrajX_[output_index_];
  output_.left_foot.y = solution_.state_vec[0].leftFootTrajY_[output_index_];
  output_.left_foot.z = solution_.state_vec[0].leftFootTrajZ_[output_index_];
  output_.left_foot.yaw = solution_.state_vec[0].leftFootTrajYaw_[output_index_];
  output_.left_foot.dx = solution_.state_vec[1].leftFootTrajX_[output_index_];
  output_.left_foot.dy = solution_.state_vec[1].leftFootTrajY_[output_index_];
  output_.left_foot.dz = solution_.state_vec[1].leftFootTrajZ_[output_index_];
  output_.left_foot.dyaw = solution_.state_vec[1].leftFootTrajYaw_[output_index_];
  output_.left_foot.ddx = solution_.state_vec[2].leftFootTrajX_[output_index_];
  output_.left_foot.ddy = solution_.state_vec[2].leftFootTrajY_[output_index_];
  output_.left_foot.ddz = solution_.state_vec[2].leftFootTrajZ_[output_index_];
  output_.left_foot.ddyaw = solution_.state_vec[2].leftFootTrajYaw_[output_index_];

  output_.right_foot.x = solution_.state_vec[0].rightFootTrajX_[output_index_];
  output_.right_foot.y = solution_.state_vec[0].rightFootTrajY_[output_index_];
  output_.right_foot.z = solution_.state_vec[0].rightFootTrajZ_[output_index_];
  output_.right_foot.yaw = solution_.state_vec[0].rightFootTrajYaw_[output_index_];
  output_.right_foot.dx = solution_.state_vec[1].rightFootTrajX_[output_index_];
  output_.right_foot.dy = solution_.state_vec[1].rightFootTrajY_[output_index_];
  output_.right_foot.dz = solution_.state_vec[1].rightFootTrajZ_[output_index_];
  output_.right_foot.dyaw = solution_.state_vec[1].rightFootTrajYaw_[output_index_];
  output_.right_foot.ddx = solution_.state_vec[2].rightFootTrajX_[output_index_];
  output_.right_foot.ddy = solution_.state_vec[2].rightFootTrajY_[output_index_];
  output_.right_foot.ddz = solution_.state_vec[2].rightFootTrajZ_[output_index_];
  output_.right_foot.ddyaw = solution_.state_vec[2].rightFootTrajYaw_[output_index_];
}

void Walkgen::reference(double dx, double dy, double dyaw){
  newVelRef_.local.x.fill(dx);
  newVelRef_.local.y.fill(dy);
  newVelRef_.local.yaw.fill(dyaw);
}

void Walkgen::reference(Eigen::VectorXd dx, Eigen::VectorXd dy, Eigen::VectorXd dyaw){
  newVelRef_.local.x=dx;
  newVelRef_.local.y=dy;
  newVelRef_.local.yaw=dyaw;
}


const SupportState & Walkgen::currentSupportState() const {
  return robot_->currentSupport(); }


const BodyState & Walkgen::bodyState(BodyType body)const{
  return robot_->body(body)->state();
}

void Walkgen::bodyState(BodyType body, const BodyState & state){
  robot_->body(body)->state(state);
}

