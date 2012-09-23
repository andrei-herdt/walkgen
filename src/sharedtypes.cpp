#include <mpc-walkgen/sharedtypes.h>
#include <mpc-walkgen/tools.h>

#include <iostream>
#include <cassert>

using namespace std;
using namespace MPCWalkgen;

Frame::Frame()
:x(1),y(1),yaw(1)
{
  x.fill(0);
  y.fill(0);
  yaw.fill(0);
}

void Frame::resize(int size){
  x.setZero(size);
  y.setZero(size);
  yaw.setZero(size);
}

Reference::Reference()
:global()
,local()
{}

void Reference::resize(int size){
  global.resize(size);
  local.resize(size);
}

BodyState::BodyState(){
  reset();
}

void BodyState::reset(){
  x.fill(0);
  y.fill(0);
  z.fill(0);
  yaw.fill(0);
  pitch.fill(0);
  roll.fill(0);
}

FootData::FootData()
: soleWidth(0)
, soleHeight(0)
, anklePositionInLocalFrame() {
  edges_x_vec.resize(4);
  edges_y_vec.resize(4);

  position.resize(3);
}

FootData::FootData(const FootData &f)
: soleWidth(f.soleWidth)
, soleHeight(f.soleHeight)
, anklePositionInLocalFrame(f.anklePositionInLocalFrame) {
  edges_x_vec = f.edges_x_vec;
  edges_y_vec = f.edges_y_vec;
  position = f.position;
}

FootData::~FootData(){
}

void FootData::SetEdges(double front, double back, double left, double right,
                        double security_margin) 
{
  // Clockwise, starting at front-left corner
  // Foots sagittal plane is aligned with the x-axis
  edges_x_vec[0] = front - security_margin;
  edges_x_vec[1] = front - security_margin;
  edges_x_vec[2] = back + security_margin;
  edges_x_vec[3] = back + security_margin;

  edges_y_vec[0] = left - security_margin;
  edges_y_vec[1] = right + security_margin;
  edges_y_vec[2] = right + security_margin;
  edges_y_vec[3] = left - security_margin;
}

HipYawData::HipYawData()
:lowerBound(-0.523599)
,upperBound(0.785398)
,lowerVelocityBound(-3.54108)
,upperVelocityBound(3.54108)
,lowerAccelerationBound(-0.1)
,upperAccelerationBound(0.1) {}

HipYawData::~HipYawData() {}

SolutionAnalysis::SolutionAnalysis():
num_iterations(-1),
objective_value(-1),
resolution_time(-1)
{}

MPCSolution::MPCSolution()
:state_vec(3)
{}

MPCSolution::~MPCSolution() {}


MPCSolution& MPCSolution::operator = (MPCSolution const &rhs){
  qp_solution_vec = rhs.qp_solution_vec;
  initialSolution = rhs.initialSolution;

  constraints = rhs.constraints;
  initialConstraints = rhs.initialConstraints;

  /// \brief True if a new trajectory is computed in online loop
  newTraj = rhs.newTraj;

  /// \brief Sampling times
  /// starting with 0, i.e. all times are relative to the current time
  samplingTimes_vec = rhs.samplingTimes_vec;

  support_states_vec = rhs.support_states_vec;

  supportOrientations_vec = rhs.supportOrientations_vec;//TODO: supportOrientations_vec
  supportTrunkOrientations_vec = rhs.supportTrunkOrientations_vec;//TODO: TrunkOrientations_vec

  CoPTrajX = rhs.CoPTrajX;
  CoPTrajY = rhs.CoPTrajY;

  com_prw = rhs.com_prw;
  cop_prw = rhs.cop_prw; 
  left_foot_prw = rhs.left_foot_prw; 
  right_foot_prw = rhs.right_foot_prw; 

  com_act = rhs.com_act;
  cop_prw = rhs.cop_act; 
  left_foot_act = rhs.left_foot_act; 
  right_foot_act = rhs.right_foot_act; 

  analysis = rhs.analysis;

  pos_ref = rhs.pos_ref;

  state_vec = rhs.state_vec;
  return(*this);
}


void MPCSolution::reset(){
  support_states_vec.resize(0);
  supportOrientations_vec.resize(0);
  supportTrunkOrientations_vec.resize(0);
}

void Motion::resize(int size) {
  pos.x_vec.setZero(size); pos.y_vec.setZero(size); pos.z_vec.setZero(size); pos.yaw_vec.setZero(size);
  vel.x_vec.setZero(size); vel.y_vec.setZero(size); vel.z_vec.setZero(size); vel.yaw_vec.setZero(size);
  acc.x_vec.setZero(size); acc.y_vec.setZero(size); acc.z_vec.setZero(size); acc.yaw_vec.setZero(size);
  jerk.x_vec.setZero(size); jerk.y_vec.setZero(size); jerk.z_vec.setZero(size); jerk.yaw_vec.setZero(size);
}


MPCData::MPCData()
:period_qpsample(0.1)
,period_mpcsample(0.005)
,period_actsample(0.005)
,num_samples_horizon(16)
,nbqpsamples_step(8)
,nbqpsamples_dsss(8)
,nbsteps_ssds(2)
,period_ds(1000000000.0)
,warmstart(false)
,interpolate_whole_horizon(false)
,closed_loop(false)
,dynamics_order(THIRD_ORDER)
,weight_coefficients(2)
{}

MPCData::~MPCData(){}

int MPCData::nbFeedbackSamplesLeft(double firstIterationduration) const{
  return static_cast<int> (round(firstIterationduration / period_mpcsample)-1 );
}

int MPCData::nbFeedbackSamplesStandard() const{
  return static_cast<int> (round(period_qpsample / period_mpcsample) );
}

int MPCData::num_samples_act() const{
  return static_cast<int> (round(period_mpcsample / period_actsample) );
}

int MPCData::num_qpsamples_ss() const {
  return nbqpsamples_step - 1;
}

int MPCData::num_steps_max() const {
  return static_cast<int>(num_samples_horizon / nbqpsamples_step) + 1;
}

double MPCData::period_ss() const {
  return nbqpsamples_step * period_qpsample - period_trans_ds();
}

double MPCData::period_trans_ds() const {
  return period_qpsample;
}

ConvexHull::ConvexHull()
:x(1)
,y(1)
,z(1)
,A(1)
,B(1)
,C(1)
,D(1) {
}

ConvexHull::~ConvexHull() {}

// TODO: Necessary?
ConvexHull &ConvexHull::operator=(const ConvexHull &hull){
  x = hull.x;
  y = hull.y;
  z = hull.z;
  A = hull.A;
  B = hull.B;
  C = hull.C;
  D = hull.D;

  return *this;
}

void ConvexHull::resize(int size){
  if (size != x.rows()){
    x.setZero(size);
    y.setZero(size);
    A.setZero(size);
    B.setZero(size);
    C.setZero(size);
    D.setZero(size);
  }else{
    x.fill(0);
    y.fill(0);
    A.fill(0);
    B.fill(0);
    C.fill(0);
    D.fill(0);
  }
}

void ConvexHull::rotate(double yaw) {
  if (fabs(yaw) < EPSILON)
    return;

  double xOld, yOld;
  int size = x.rows();
  for(int i = 0; i < size; ++i){
    xOld = x(i);
    yOld = y(i);
    x(i) = (xOld * std::cos(yaw) - yOld * std::sin(yaw));
    y(i) = (xOld * std::sin(yaw) + yOld * std::cos(yaw));
  }
}

void ConvexHull::computeLinearSystem(const Foot &foot) {
  double dx,dy,dc,x1,y1,x2,y2;
  unsigned nbRows = x.rows();

  double sign;
  if(foot == LEFT){
    sign = 1.0;
  }else{
    sign = -1.0;
  }
  for( unsigned i=0; i<nbRows;++i ){
    y1 = y(i);
    y2 = y((i+1)%nbRows);
    x1 = x(i);
    x2 = x((i+1)%nbRows);

    dx = sign*(y1-y2);
    dy = sign*(x2-x1);
    dc = dx*x1+dy*y1;


    A(i) = dx;
    B(i) = dy;
    D(i) = dc;
  }
}

RobotData::RobotData(const FootData &leftFoot, const FootData &rightFoot,
                     const HipYawData &leftHipYaw, const HipYawData &rightHipYaw,
                     double mass)
                     :	com()
                     ,max_foot_height(0.03)
                     ,leftFoot(leftFoot)
                     ,rightFoot(rightFoot)
                     ,leftHipYaw(leftHipYaw)
                     ,rightHipYaw(rightHipYaw)
                     ,mass(mass)
                     ,max_foot_vel(0.)
                     ,leftFootHull()
                     ,rightFootHull()
                     ,CoPLeftSSHull()
                     ,CoPRightSSHull()
                     ,CoPLeftDSHull()
                     ,CoPRightDSHull() {
                       CoPLeftSSHull.resize(4);
                       CoPRightSSHull.resize(4);
                       CoPLeftDSHull.resize(4);
                       CoPRightDSHull.resize(4);
}

RobotData::RobotData(){
  CoPLeftSSHull.resize(4);
  CoPRightSSHull.resize(4);
  CoPLeftDSHull.resize(4);
  CoPRightDSHull.resize(4);
}
RobotData::~RobotData(){}

// TODO: This function should be moved to a separate object for hulls/constraints
void RobotData::SetCoPHulls(double ds_distance) {
  for (int i = 0; i < 4; ++i) {
    CoPLeftSSHull.x(i) = leftFoot.edges_x_vec[i];
    CoPLeftSSHull.y(i) = leftFoot.edges_y_vec[i];
    CoPLeftDSHull.x(i) = leftFoot.edges_x_vec[i];
    CoPLeftDSHull.y(i) = leftFoot.edges_y_vec[i];

    CoPRightSSHull.x(i) = rightFoot.edges_x_vec[i];
    CoPRightSSHull.y(i) = -rightFoot.edges_y_vec[i]; // Counter-clockwise
    CoPRightDSHull.x(i) = rightFoot.edges_x_vec[i];
    CoPRightDSHull.y(i) = -rightFoot.edges_y_vec[i]; // Counter-clockwise
  }
  CoPLeftDSHull.y(1)  -= ds_distance;
  CoPLeftDSHull.y(2)  -= ds_distance;
  CoPRightDSHull.y(1) += ds_distance;
  CoPRightDSHull.y(2) += ds_distance;
}

WeightCoefficients::WeightCoefficients(int num_modes)
:pos(num_modes)
,vel(num_modes)
,acc(num_modes)
,jerk(num_modes)
,cop(num_modes) {
  pos[0] =  0.;
  cop[0]  = 0.0001;
  jerk[0] = 0.00001;
  vel[0]  = 1;

  pos[1] =  0.;
  cop[1]  = 1.;
  jerk[1] = 0.01;
  vel[1]  = 1.;

  active_mode  = 0;

  is_initial_mode = true;
}
WeightCoefficients::~WeightCoefficients(){}

void WeightCoefficients::SetCoefficients(Reference &ref) { 
  if (fabs(ref.local.yaw(0)) < kEps && fabs(ref.local.x(0)) < kEps && fabs(ref.local.y(0)) < kEps) {
    if (is_initial_mode) {
      active_mode = 0;
    } else {
      active_mode = 1;
    }
  } else {
    active_mode = 0;
    is_initial_mode = false;
  }
}


