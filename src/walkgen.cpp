#include <mpc-walkgen/walkgen.h>

#include <mpc-walkgen/orientations-preview.h>
#include <mpc-walkgen/qp-solver.h>
#include <mpc-walkgen/qp-generator.h>
#include <mpc-walkgen/qp-preview.h>
#include <mpc-walkgen/rigid-body-system.h>
#include <mpc-walkgen/interpolation.h>
#include <mpc-walkgen/mpc-debug.h>
 
#include <iostream>
#include <windows.h> 
#include <Eigen/Dense>
#include <stdio.h>

using namespace MPCWalkgen;

using namespace Eigen;


MPCWalkgen::WalkgenAbstract* MPCWalkgen::createWalkgen() {
  MPCWalkgen::WalkgenAbstract* zmpVra = new MPCWalkgen::Walkgen();
  return zmpVra;
}




// Implementation of the private interface
Walkgen::Walkgen()
: WalkgenAbstract()
,data_mpc_()
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

void Walkgen::Init(const RobotData &data_robot, const MPCData &mpc_data) {

  data_mpc_ = mpc_data;
  robotData_ = data_robot;

  // Check if sampling periods are defined correctly
  assert(data_mpc_.period_actsample > 0);
  assert(data_mpc_.period_mpcsample >= data_mpc_.period_actsample);
  assert(data_mpc_.period_qpsample >= data_mpc_.period_mpcsample);
  assert(data_mpc_.nbqpsamples_step <= data_mpc_.nbsamples_qp);
  assert(data_mpc_.nbqpsamples_dsss <= data_mpc_.nbsamples_qp);

  Init();

}

void Walkgen::Init() {
  //TODO: Cleaning is necessary
  int num_steps_max = data_mpc_.num_steps_max();
  int num_constr_step = 5;
  solver_ = createQPSolver(data_mpc_.solver,
    2 * (data_mpc_.nbsamples_qp + num_steps_max), 
    num_constr_step * num_steps_max  );

  orientPrw_ = new OrientationsPreview();

  interpolation_ = new Interpolation();

  robot_ = new RigidBodySystem(&data_mpc_);

  preview_ = new QPPreview(&velRef_, robot_, &data_mpc_);

  generator_= new QPGenerator(preview_, solver_, &velRef_, &ponderation_, robot_, &data_mpc_);

  robot_->Init(robotData_, interpolation_);

  solution_.com_act.resize(data_mpc_.num_samples_act());
  solution_.cop_act.resize(data_mpc_.num_samples_act());
  solution_.com_prw.resize(data_mpc_.nbsamples_qp);
  solution_.cop_prw.resize(data_mpc_.nbsamples_qp);

  // Redistribute the X,Y vectors of variables inside the optimization problems
  VectorXi order(solver_->nbvar_max());
  for (int i = 0; i < data_mpc_.nbsamples_qp; ++i) {// 0,2,4,1,3,5 (CoM)
    order(i) = 2*i;
    order(i + data_mpc_.nbsamples_qp) = 2*i+1;
  }

  for (int i = 2*data_mpc_.nbsamples_qp; i < solver_->nbvar_max(); ++i) {// 6,7,8 (Feet)
    order(i) = i;
  }

  solver_->varOrder(order);

  orientPrw_->Init(data_mpc_, robotData_);

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

  velRef_.resize(data_mpc_.nbsamples_qp);
  newVelRef_.resize(data_mpc_.nbsamples_qp);

  BuildProblem();

  solver_->Init();
}

const MPCSolution &Walkgen::online(){
  currentRealTime_ += data_mpc_.period_mpcsample;
  return online(currentRealTime_);
}

const MPCSolution &Walkgen::online(double time){
  currentTime_ = time;

  if (time  > next_computation_ - EPSILON) {
    next_computation_ += data_mpc_.period_mpcsample;
    if (time > next_computation_ - EPSILON) {   
      ResetCounters(time);
    }
    if(time  > first_sample_time_ - EPSILON){
      first_sample_time_ += data_mpc_.period_qpsample;
      if (time > first_sample_time_ - EPSILON) {
        ResetCounters(time);
      }
    }
    ResetOutputIndex();

    BuildProblem();

    solver_->Solve(solution_,
      data_mpc_.warmstart, data_mpc_.analyze_resolution);

    GenerateTrajectories();
  }

  if (time > next_act_sample_ - EPSILON) {
    next_act_sample_ += data_mpc_.period_actsample;

    IncrementOutputIndex();
    UpdateOutput();
  }

  return solution_;
}

void Walkgen::SetCounters(double time) {

}
void Walkgen::ResetCounters(double time) {
  first_sample_time_ = time + data_mpc_.period_qpsample;
  next_computation_ = time + data_mpc_.period_mpcsample;
  next_act_sample_ = time + data_mpc_.period_actsample;
}

void Walkgen::BuildProblem() {
  // UPDATE INTERNAL DATA:
  // ---------------------
  solver_->reset();
  solution_.reset();
  velRef_ = newVelRef_;
  if (isNewCurrentSupport_){
    robot_->current_support(newCurrentSupport_);
    isNewCurrentSupport_ = false;
  }

  if (robot_->current_support().phase == SS && robot_->current_support().nbStepsLeft == 0) {
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
    data_mpc_.nbqpsamples_step * data_mpc_.period_qpsample,
    robot_->body(LEFT_FOOT)->state(), robot_->body(RIGHT_FOOT)->state(),
    solution_ );
  preview_->computeRotationMatrix(solution_);

  generator_->computeReferenceVector(solution_);

  // BUILD:
  // ------
  generator_->BuildProblem(solution_);
}

void Walkgen::GenerateTrajectories() {
  generator_->ConvertCopToJerk(solution_);

  robot_->interpolateBodies(solution_, currentTime_, velRef_);

  robot_->UpdateState(solution_);

  orientPrw_->interpolate_trunk_orientation(robot_);

  ResetOutputIndex();
  UpdateOutput();
}


void Walkgen::ResetOutputIndex() {
  output_index_ = 0;
}

void Walkgen::IncrementOutputIndex() {
  if (output_index_ < data_mpc_.num_samples_act() - 1) {
    output_index_++;
  }
}

void Walkgen::UpdateOutput() {
  output_.com.x = solution_.com_act.pos.x_vec[output_index_];
  output_.com.y = solution_.com_act.pos.y_vec[output_index_];
  output_.com.z = robot_->body(COM)->state().z(0);
  output_.com.dx = solution_.com_act.vel.x_vec[output_index_];
  output_.com.dy = solution_.com_act.vel.y_vec[output_index_];
  output_.com.dz = robot_->body(COM)->state().z(1);
  output_.com.ddx = solution_.com_act.acc.x_vec[output_index_];
  output_.com.ddy = solution_.com_act.acc.y_vec[output_index_];
  output_.com.ddz = robot_->body(COM)->state().z(2);

  output_.cop.x =   solution_.cop_act.pos.x_vec[output_index_];
  output_.cop.y =   solution_.cop_act.pos.y_vec[output_index_];

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


const SupportState &Walkgen::currentSupportState() const {
  return robot_->current_support(); }


const BodyState &Walkgen::bodyState(BodyType body)const{
  return robot_->body(body)->state();
}

void Walkgen::bodyState(BodyType body, const BodyState &state){
  robot_->body(body)->state(state);
}
