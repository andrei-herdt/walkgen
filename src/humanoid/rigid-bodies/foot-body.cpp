#include "foot-body.h"
#include "../../common/tools.h"

using namespace MPCWalkgen;

using namespace Eigen;


FootBody::FootBody(const MPCData *data_mpc,
                   const RobotData *data_robot, Foot type)
                   :RigidBody(data_mpc, data_robot)
                   ,footType_(type)
{}

FootBody::~FootBody(){}

void FootBody::Interpolate(MPCSolution &solution, double currentTime, const Reference &/*velRef*/) {

  BodyState nextFootState;
  SupportState curSupport = solution.supportStates_vec.front();
  SupportState nextSupportState = solution.supportStates_vec[1];

  double time_left_xy = 1; // Duration of the current interpolation phase of the horizontal motion
  double time_left_z = 1; // Duration of the current interpolation phase of the vertical motion
  int nbsamples = data_mpc_->nbSamplesControl();
  double period_ds = data_mpc_->period_trans_ds();
  double raise_period = 0.05; // Time during which the horizontal displacement is blocked
  double time_left_flying = curSupport.timeLimit - period_ds - currentTime;
  double time_spent_flying = currentTime - curSupport.startTime;
  if (solution.supportStates_vec[0].phase == SS) {
    int nbStepsPreviewed = solution.supportStates_vec.back().stepNumber;
    if (nextSupportState.inTransitionalDS) {
      time_left_xy = curSupport.timeLimit - currentTime;
      nextFootState.x(0) = state_.x(0);
      nextFootState.y(0) = state_.y(0);
      nextFootState.yaw(0) = state_.yaw(0);
    } else if (time_spent_flying < raise_period + EPSILON) {
      time_left_xy = raise_period - time_spent_flying;
      nextFootState.x(0) = state_.x(0);
      nextFootState.y(0) = state_.y(0);
      nextFootState.yaw(0) = state_.yaw(0);
    }  else if (time_left_flying < raise_period + EPSILON) {
      time_left_xy = time_left_flying;
      nextFootState.x(0) = state_.x(0);
      nextFootState.y(0) = state_.y(0);
      nextFootState.yaw(0) = state_.yaw(0);
    } else {
      time_left_xy = time_left_flying - raise_period;
      int nbPreviewedSteps = solution.supportStates_vec.back().stepNumber;
      if (nbPreviewedSteps > 0) {
        nextFootState.x(0) = solution.qpSolution(2 * data_mpc_->nbsamples_qp);
        nextFootState.y(0) = solution.qpSolution(2 * data_mpc_->nbsamples_qp + nbStepsPreviewed);
        nextFootState.yaw(0) = solution.supportOrientations_vec[0];
      } else {
        nextFootState.x(0) = state_.x(0);
        nextFootState.y(0) = state_.y(0);
        nextFootState.yaw(0) = state_.yaw(0);
      }
    }
    if (time_left_flying - (data_mpc_->period_ss() / 2.0) > data_mpc_->period_actsample) {
      nextFootState.z(0) = robotData_->freeFlyingFootMaxHeight;
      time_left_z = time_left_flying - (data_mpc_->period_ss() / 2.0) ;
    } else if (time_left_flying < (data_mpc_->period_ss() / 2.0) && time_left_flying > EPSILON) { // Half-time passed
      time_left_z = time_left_flying;
    } else {
      // time_left_z stays 1
    }
  }

  InterpolatePolynomial(solution, X, nbsamples, time_left_xy, state_.x, nextFootState.x);
  InterpolatePolynomial(solution, Y, nbsamples, time_left_xy, state_.y, nextFootState.y);
  InterpolatePolynomial(solution, Z, nbsamples, time_left_z, state_.z, nextFootState.z);
  InterpolatePolynomial(solution, Yaw, nbsamples, time_left_xy, state_.yaw, nextFootState.yaw);

}

//TODO: Remove this ...
void FootBody::ComputeDynamicsMatrices(LinearDynamicsMatrices &dyn,
                                       double sample_period_first, double sample_period_rest, int nbsamples, Derivative type) 
{ }



VectorXd &FootBody::getFootVector(MPCSolution &solution, Axis axis, unsigned derivative) {
  MPCSolution::State &currentState = solution.state_vec[derivative];
  if (footType_ == LEFT){
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

void FootBody::InterpolatePolynomial(MPCSolution &solution, Axis axis, int nbsamples, double T, 
                                                    const Eigen::Vector3d &init_state_vec,
                                                    const Eigen::Vector3d &final_state_vec) 
{
  VectorXd &FootTrajState = getFootVector(solution, axis, 0);
  VectorXd &FootTrajVel   = getFootVector(solution, axis, 1);
  VectorXd &FootTrajAcc   = getFootVector(solution, axis, 2);

  if (FootTrajState.rows() != nbsamples) {
    FootTrajState.resize(nbsamples);
    FootTrajVel.resize(nbsamples);
    FootTrajAcc.resize(nbsamples);
  }

  if (solution.supportStates_vec[0].foot == footType_) {
    FootTrajState.fill(init_state_vec(0));
    FootTrajVel.fill(0.0);
    FootTrajAcc.fill(0.0);

  } else {
    if (solution.supportStates_vec[0].phase == DS) {
      FootTrajState.fill(init_state_vec(0));
      FootTrajVel.fill(0.0);
      FootTrajAcc.fill(0.0);

    }else{
      Eigen::Matrix<double,6,1> factor;
      interpolation_.computePolynomialNormalisedFactors(factor, init_state_vec, final_state_vec, T);
      for (int i = 0; i < nbsamples; ++i) {
        double ti = (i+1)*data_mpc_->period_actsample;

        FootTrajState(i) = p(factor, ti/T);
        FootTrajVel(i)   = dp(factor, ti/T)/T;
        FootTrajAcc(i)   = ddp(factor, ti/T)/(T*T);
      }
    }
  }
}
