#include "foot-body.h"
#include "../../common/tools.h"

using namespace MPCWalkgen;

using namespace Eigen;


FootBody::FootBody(const MPCData *generalData,
		   const RobotData *robotData, Foot type)
  :RigidBody(generalData, robotData)
  ,footType_(type)
{}

FootBody::~FootBody(){}

void FootBody::Interpolate(MPCSolution &solution, double currentTime, const Reference &/*velRef*/) {

  BodyState nextFootState;
  SupportState curSupport = solution.supportStates_vec.front();
  SupportState nextSupportState = solution.supportStates_vec[1];

  double Txy = 1; // Local horizontal interpolation time
  double Tz = 1; // Local vertical interpolation time
  int nbSamples = generalData_->nbSamplesControl();
  double period_ds = generalData_->period_trans_ds();
  double raiseTime = 0.05; // Time during which the horizontal displacement is blocked
  double freeFlyingTimeLeft = curSupport.timeLimit - period_ds - currentTime;
  double freeFlyingTimeSpent = currentTime - curSupport.startTime;
  if (solution.supportStates_vec[0].phase == SS) {
      int nbStepsPreviewed = solution.supportStates_vec.back().stepNumber;
      if (nextSupportState.inTransitionalDS) {
          Txy = curSupport.timeLimit - currentTime;
          nextFootState.x(0) = state_.x(0);
          nextFootState.y(0) = state_.y(0);
          nextFootState.yaw(0) = state_.yaw(0);
        } else if (freeFlyingTimeSpent < raiseTime + EPSILON) {
          Txy = raiseTime - freeFlyingTimeSpent;
          nextFootState.x(0) = state_.x(0);
          nextFootState.y(0) = state_.y(0);
          nextFootState.yaw(0) = state_.yaw(0);
        }  else if (freeFlyingTimeLeft < raiseTime + EPSILON) {
          Txy = freeFlyingTimeLeft;
          nextFootState.x(0) = state_.x(0);
          nextFootState.y(0) = state_.y(0);
          nextFootState.yaw(0) = state_.yaw(0);
        } else {
          Txy = freeFlyingTimeLeft - raiseTime;
          int nbPreviewedSteps = solution.supportStates_vec.back().stepNumber;
          if (nbPreviewedSteps > 0) {
              nextFootState.x(0) = solution.qpSolution(2 * generalData_->nbsamples_qp);
              nextFootState.y(0) = solution.qpSolution(2 * generalData_->nbsamples_qp + nbStepsPreviewed);
              nextFootState.yaw(0) = solution.supportOrientations_vec[0];
            } else {
              nextFootState.x(0) = state_.x(0);
              nextFootState.y(0) = state_.y(0);
              nextFootState.yaw(0) = state_.yaw(0);
            }
        }
      if (freeFlyingTimeLeft - (generalData_->period_ss() / 2.0) > generalData_->period_actsample) {
          nextFootState.z(0) = robotData_->freeFlyingFootMaxHeight;
          Tz = freeFlyingTimeLeft - (generalData_->period_ss() / 2.0) ;
        } else if (freeFlyingTimeLeft < (generalData_->period_ss() / 2.0) && freeFlyingTimeLeft > EPSILON) { // Half-time passed
          Tz = freeFlyingTimeLeft;
        } else {
          // Tz stays 1
        }
    }

  computeFootInterpolationByPolynomial(solution, X, nbSamples,
                                       state_.x,
                                       Txy, nextFootState.x);
  computeFootInterpolationByPolynomial(solution, Y, nbSamples,
                                       state_.y,
                                       Txy, nextFootState.y);
  computeFootInterpolationByPolynomial(solution, Z, nbSamples,
                                       state_.z,
                                       Tz, nextFootState.z);
  computeFootInterpolationByPolynomial(solution, Yaw, nbSamples,
                                       state_.yaw,
                                       Txy, nextFootState.yaw);

}

//TODO: Remove this ...
void FootBody::ComputeDynamicsMatrices(LinearDynamicsMatrices &dyn,
                                      double sample_period_first, double sample_period_rest, int nbsamples, Derivative type) 
{ }



VectorXd &FootBody::getFootVector(MPCSolution &solution, Axis axis, unsigned derivative) {
  MPCSolution::State &currentState = solution.state_vec[derivative];
  if (footType_==LEFT){
      switch(axis){
        case X:
          return currentState.leftFootTrajX_;
        case Y:
          return currentState.leftFootTrajY_;
        case Z:
          return currentState.leftFootTrajZ_;
        default:
          return currentState.leftFootTrajYaw_;
        }
    }else{
      switch(axis){
        case X:
          return currentState.rightFootTrajX_;
        case Y:
          return currentState.rightFootTrajY_;
        case Z:
          return currentState.rightFootTrajZ_;
        default:
          return currentState.rightFootTrajYaw_;
        }
    }
}

void FootBody::computeFootInterpolationByPolynomial(MPCSolution &solution, Axis axis, int nbSamples,
						    const Eigen::Vector3d &FootCurrentState,
						    double T, const Eigen::Vector3d &nextSupportFootState){

  VectorXd &FootTrajState = getFootVector(solution, axis, 0);
  VectorXd &FootTrajVel   = getFootVector(solution, axis, 1);
  VectorXd &FootTrajAcc   = getFootVector(solution, axis, 2);

  if (FootTrajState.rows() != nbSamples){
      FootTrajState.resize(nbSamples);
      FootTrajVel.resize(nbSamples);
      FootTrajAcc.resize(nbSamples);
    }

  if (solution.supportStates_vec[0].foot==footType_){
      FootTrajState.fill(FootCurrentState(0));
      FootTrajVel.fill(0);
      FootTrajAcc.fill(0);

    }else{

      if (solution.supportStates_vec[0].phase == DS){
          FootTrajState.fill(FootCurrentState(0));
          FootTrajVel.fill(0);
          FootTrajAcc.fill(0);

        }else{
          Eigen::Matrix<double,6,1> factor;
          interpolation_.computePolynomialNormalisedFactors(factor, FootCurrentState, nextSupportFootState, T);
          for (int i = 0; i < nbSamples; ++i) {
              double ti = (i+1)*generalData_->period_actsample;

              FootTrajState(i) = p(factor, ti/T);
              FootTrajVel(i)   = dp(factor, ti/T)/T;
              FootTrajAcc(i)   = ddp(factor, ti/T)/(T*T);

            }
        }


    }
}
