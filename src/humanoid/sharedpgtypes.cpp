#include <mpc-walkgen/humanoid/sharedpgtypes.h>
#include "../common/tools.h"
#include <iostream>
#include <cassert>
using namespace std;
using namespace MPCWalkgen;


FootData::FootData()
: soleWidth(0)
, soleHeight(0)
, anklePositionInLocalFrame(){}

FootData::FootData(const FootData &f)
: soleWidth(f.soleWidth)
, soleHeight(f.soleHeight)
, anklePositionInLocalFrame(f.anklePositionInLocalFrame){}//TODO: LocalAnklePosition_ better?

FootData::~FootData(){}

HipYawData::HipYawData()
:lowerBound(-0.523599)
,upperBound(0.785398)
,lowerVelocityBound(-3.54108)
,upperVelocityBound(3.54108)
,lowerAccelerationBound(-0.1)
,upperAccelerationBound(0.1) {}

HipYawData::~HipYawData() {}

MPCSolution::MPCSolution()
:state_vec(3)
{}

MPCSolution::~MPCSolution() {}


MPCSolution& MPCSolution::operator = (MPCSolution const &rhs){
  qpSolution = rhs.qpSolution;
  initialSolution = rhs.initialSolution;

  constraints = rhs.constraints;
  initialConstraints = rhs.initialConstraints;
  /// \brief True if a new trajectory is computed in online loop
  newTraj = rhs.newTraj;

  /// \brief Sampling times
  /// starting with 0, i.e. all times are relative to the current time
  samplingTimes_vec = rhs.samplingTimes_vec;

  supportStates_vec = rhs.supportStates_vec;

  supportOrientations_vec = rhs.supportOrientations_vec;//TODO: supportOrientations_vec
  supportTrunkOrientations_vec = rhs.supportTrunkOrientations_vec;//TODO: TrunkOrientations_vec

  CoPTrajX = rhs.CoPTrajX;
  CoPTrajY = rhs.CoPTrajY;

  com_prw = rhs.com_prw;
  cop_prw = rhs.cop_prw; 
  foot_left_prw = rhs.foot_left_prw; 
  foot_right_prw = rhs.foot_right_prw; 

  state_vec = rhs.state_vec;
  return(*this);
}

void MPCSolution::reset(){
  supportStates_vec.resize(0);
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
,nbsamples_qp(16)
,nbqpsamples_step(8)
,nbqpsamples_dsss(8)
,nbsteps_ssds(2)
,period_ds(1000000000.0)
,ponderation(2)
,warmstart(false)
,interpolate_preview(false){
}

MPCData::~MPCData(){}

int MPCData::nbFeedbackSamplesLeft(double firstIterationduration) const{
  return static_cast<int> (round(firstIterationduration / period_mpcsample)-1 );
}

int MPCData::nbFeedbackSamplesStandard() const{
  return static_cast<int> (round(period_qpsample / period_mpcsample) );
}

int MPCData::nbSamplesControl() const{
  return static_cast<int> (round(period_mpcsample / period_actsample) );
}

double MPCData::period_ss() const {
  return nbqpsamples_step * period_qpsample - period_trans_ds();
}

double MPCData::period_trans_ds() const {
  return period_qpsample;
}

RobotData::RobotData(const FootData &leftFoot, const FootData &rightFoot,
                     const HipYawData &leftHipYaw, const HipYawData &rightHipYaw,
                     double mass)
                     :	com()
                     ,freeFlyingFootMaxHeight(0.05)
                     ,leftFoot(leftFoot)
                     ,rightFoot(rightFoot)
                     ,leftHipYaw(leftHipYaw)
                     ,rightHipYaw(rightHipYaw)
                     ,robotMass(mass)
                     ,leftFootPos()
                     ,rightFootPos()
                     ,leftFootHull()
                     ,rightFootHull()
                     ,CoPLeftSSHull()
                     ,CoPRightSSHull()
                     ,CoPLeftDSHull()
                     ,CoPRightDSHull() {
                       com << 0.0, 0.0, 0.814;
                       leftFootPos << 0.00949035, 0.095, 0;
                       rightFootPos << 0.00949035, -0.095, 0;
}

RobotData::RobotData(){}
RobotData::~RobotData(){}

QPPonderation::QPPonderation(int nb)
:instantVelocity(nb)
,CopCentering(nb)
,JerkMin(nb) {
  CopCentering[0]    = 0.0001;
  JerkMin[0]         = 0.00001;
  instantVelocity[0] = 1;

  CopCentering[1]    = 0.0001;
  JerkMin[1]         = 0.00001;
  instantVelocity[1] = 1;

  activePonderation  = 1;

}
QPPonderation::~QPPonderation(){}


