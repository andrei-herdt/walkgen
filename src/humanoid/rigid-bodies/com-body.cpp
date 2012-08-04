#include "com-body.h"
#include "../../common/tools.h"

using namespace MPCWalkgen;

using namespace Eigen;


CoMBody::CoMBody(const MPCData *generalData,
		 const RobotData *robotData,
		 const Interpolation *interpolation)
  :RigidBody(generalData, robotData, interpolation)
{}
//TODO: Can CoMBody be merged with FootBody?
CoMBody::~CoMBody(){}

void CoMBody::interpolate(MPCSolution &solution, double currentTime, const Reference &velRef){
  // Position:
  interpolation_->computeInterpolationByJerk(solution.state_vec[0].CoMTrajX_, solution.state_vec[0].CoMTrajY_, state_,
                                             dynamics(interpolationPos), solution.qpSolution(0),
                                             solution.qpSolution(generalData_->nbsamples_qp));

  // TODO: Only in debug mode
  solution.com_prw.pos.x_vec = interpolation_->Interpolate(dynamics(POSITION), 
    state_.x, solution.qpSolution.segment(0, generalData_->nbsamples_qp));
  solution.com_prw.pos.y_vec = interpolation_->Interpolate(dynamics(POSITION), 
    state_.y, solution.qpSolution.segment(0, generalData_->nbsamples_qp));

  // Velocity:
  interpolation_->computeInterpolationByJerk(solution.state_vec[1].CoMTrajX_, solution.state_vec[1].CoMTrajY_, state_,
                                             dynamics(interpolationVel), solution.qpSolution(0),
                                             solution.qpSolution(generalData_->nbsamples_qp));
    // TODO: Only in debug mode
  solution.com_prw.vel.x_vec = interpolation_->Interpolate(dynamics(VELOCITY), 
    state_.x, solution.qpSolution.segment(0, generalData_->nbsamples_qp));
  solution.com_prw.vel.y_vec = interpolation_->Interpolate(dynamics(VELOCITY), 
    state_.y, solution.qpSolution.segment(0, generalData_->nbsamples_qp));

  // Acceleration:
  interpolation_->computeInterpolationByJerk(solution.state_vec[2].CoMTrajX_, solution.state_vec[2].CoMTrajY_, state_,
                                             dynamics(interpolationAcc), solution.qpSolution(0),
                                             solution.qpSolution(generalData_->nbsamples_qp));
    // TODO: Only in debug mode
  solution.com_prw.acc.x_vec = interpolation_->Interpolate(dynamics(ACCELERATION), 
    state_.x, solution.qpSolution.segment(0, generalData_->nbsamples_qp));
  solution.com_prw.acc.y_vec = interpolation_->Interpolate(dynamics(ACCELERATION), 
    state_.y, solution.qpSolution.segment(0, generalData_->nbsamples_qp));

  // Jerk:
  interpolation_->computeInterpolationByJerk(solution.CoPTrajX, solution.CoPTrajY, state_,
                                             dynamics(interpolationCoP), solution.qpSolution(0),
                                             solution.qpSolution(generalData_->nbsamples_qp));

  interpolateTrunkOrientation(solution, currentTime, velRef);
}

void CoMBody::computeDynamicsMatrices(LinearDynamics &dyn,
                                      double S, double T, int N, DynamicMatrixType type){
  dyn.S.setZero(N,3);
  dyn.U.setZero(N,N);
  dyn.UT.setZero(N,N);
  dyn.UInv.setZero(N,N);
  dyn.UInvT.setZero(N,N);


  switch (type){
    case POSITION:
      for (int i=0; i<N; ++i) {
          dyn.S(i,0) = 1;
          dyn.S(i,1) = i*T + S;
          dyn.S(i,2) = S*S/2 + i*T*S + i*i*T*T/2;

          dyn.U(i,0) = dyn.UT(0,i) = S*S*S/6 + i*T*S*S/2 + S*(i*i*T*T/2 );
          for (int j=1; j<N; j++) {
              if (j <= i) {
                  dyn.U(i,j) = dyn.UT(j,i) =T*T*T/6 + 3*(i-j)*T*T*T/6 + 3*(i-j)*(i-j)*T*T*T/6;
                }
            }
        }
      break;

    case VELOCITY:
      for (int i=0;i<N;i++) {
          dyn.S(i,0) = 0.0;
          dyn.S(i,1) = 1.0;
          dyn.S(i,2) = i*T + S;

          dyn.U(i,0) = dyn.UT(0,i) = S*S/2 + i*T*S;
          for (int j=1; j<N; j++) {
              if (j<=i){
                  dyn.U(i,j) = dyn.UT(j,i) = T*T/2 + (i-j)*T*T;
                }
            }
        }
      break;

    case ACCELERATION:
      for (int i=0; i<N; i++) {
          dyn.S(i,2) = 1.0;

          dyn.U(i,0) = dyn.UT(0,i) = S;
          for (int j=1; j<N; j++) {
              if (j<=i){
                  dyn.U(i,j) = dyn.UT(j,i) = T;
                }
            }
        }
      break;

    case COP:
      for (int i=0; i<N; i++) {
          dyn.S(i,0) = 1;
          dyn.S(i,1) = i*T + S;
          dyn.S(i,2) = S*S/2 + i*T*S + i*i*T*T/2-robotData_->com(2)/9.81;

          dyn.U(i,0) = dyn.UT(0,i) =S*S*S/6 + i*T*S*S/2 + S*(i*i*T*T/2 - robotData_->com(2)/9.81);
          for(int j=1; j<N; j++){
              if (j <= i) {
                  dyn.U(i,j) = dyn.UT(j,i) = T*T*T/6 + 3*(i-j)*T*T*T/6 + 3*(i-j)*(i-j)*T*T*T/6 - T*robotData_->com(2)/9.81;
                }
            }

        }
      inverse(dyn.U,dyn.UInv);
      dyn.UInvT=dyn.UInv.transpose();
      break;

    default:
      dyn.U.setIdentity();
      dyn.UT.setIdentity();
      break;
    }
}

void CoMBody::interpolateTrunkOrientation(MPCSolution &result,
                                          double /*currentTime*/, const Reference &velRef){

  Eigen::Matrix<double,6,1> factor;


  double T = generalData_->nbqpsamples_step * generalData_->period_qpsample;/*
        if (result.supportState_vec[0].phase == ss){
                T = result.supportState_vec[0].startTime+generalData_->stepPeriod-currentTime;;
        }else{
                T = generalData_->stepPeriod;
        }*/


  int nbSampling = generalData_->nbSamplesControl();

  if (result.state_vec[0].trunkYaw_.rows() != nbSampling){
      for (int i = 0; i < 3; ++i) {
          result.state_vec[i].trunkYaw_.resize(nbSampling);
        }
    }

  double orientation0 = result.supportTrunkOrientations_vec[0];
  double orientation1;

  int size = result.supportTrunkOrientations_vec.size();


  if (size >= 2) {
      orientation1 = result.supportTrunkOrientations_vec[1];
    } else {
      orientation1 = orientation0;
    }

  Vector3d nextTrunkState(orientation1, -velRef.local.yaw(0), 0);


  interpolation_->computePolynomialNormalisedFactors(factor, state().yaw, nextTrunkState, T);
  for (int i=0; i < nbSampling; ++i) {
      double ti = (i+1)*generalData_->period_actsample;

      result.state_vec[0].trunkYaw_(i) = p(factor, ti/T);
      result.state_vec[1].trunkYaw_(i) = dp(factor, ti/T)/T;
      result.state_vec[2].trunkYaw_(i) = ddp(factor, ti/T)/(T*T);

    }


}
