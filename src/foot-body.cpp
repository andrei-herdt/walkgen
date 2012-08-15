#include <mpc-walkgen/foot-body.h>
#include <mpc-walkgen/tools.h>

using namespace MPCWalkgen;
using namespace Eigen;

FootBody::FootBody(const MPCData *data_mpc,
                   const RobotData *data_robot, Foot type)
                   :RigidBody(data_mpc, data_robot)
                   ,footType_(type)
{}

FootBody::~FootBody(){}

void FootBody::Interpolate(MPCSolution &solution, double currentTime, const Reference &/*velRef*/) {

  BodyState goal_state;
  const SupportState &current_support = solution.support_states_vec[0];
  const SupportState &next_support = solution.support_states_vec[1];

  double time_left_xy = 1; // Duration of the current interpolation phase of the horizontal motion
  double time_left_z = 1; // Duration of the current interpolation phase of the vertical motion
  int nbsamples = data_mpc_->num_samples_act();
  double period_ds = data_mpc_->period_trans_ds();
  double raise_period = std::max(0.05, data_mpc_->period_mpcsample); // Time during which the horizontal displacement is blocked
  double time_left_flying = 0.0;
  double time_spent_flying = 0.0;
  double halftime_rounded = std::ceil(static_cast<double>(data_mpc_->num_qpsamples_ss()) / 2.0) * data_mpc_->period_qpsample;
  if (current_support.phase == SS) {
    time_left_flying = current_support.time_limit - period_ds - currentTime;
    time_spent_flying = currentTime - current_support.start_time;
    int nbStepsPreviewed = solution.support_states_vec.back().stepNumber;
    if (next_support.transitional_ds) {
      time_left_xy = current_support.time_limit - currentTime;
      goal_state.x(0) = state_.x(0);
      goal_state.y(0) = state_.y(0);
      goal_state.yaw(0) = state_.yaw(0);
    } else if (time_spent_flying < raise_period - EPSILON) {
      time_left_xy = raise_period - time_spent_flying;
      goal_state.x(0) = state_.x(0);
      goal_state.y(0) = state_.y(0);
      goal_state.yaw(0) = state_.yaw(0);
    }  else if (time_left_flying < raise_period + EPSILON) {
      time_left_xy = time_left_flying;
      goal_state.x(0) = state_.x(0);
      goal_state.y(0) = state_.y(0);
      goal_state.yaw(0) = state_.yaw(0);
    } else {
      time_left_xy = time_left_flying - raise_period;
      int nbPreviewedSteps = solution.support_states_vec.back().stepNumber;
      if (nbPreviewedSteps > 0) {
        goal_state.x(0) = solution.qpSolution(2 * data_mpc_->nbsamples_qp);
        goal_state.y(0) = solution.qpSolution(2 * data_mpc_->nbsamples_qp + nbStepsPreviewed);
        goal_state.yaw(0) = solution.supportOrientations_vec[0];
      } else {
        goal_state.x(0) = state_.x(0);
        goal_state.y(0) = state_.y(0);
        goal_state.yaw(0) = state_.yaw(0);
      }
    }
    // Vertical trajectory
    if (time_left_flying - halftime_rounded > data_mpc_->period_actsample) {
      goal_state.z(0) = robotData_->freeFlyingFootMaxHeight;
      time_left_z = time_left_flying - halftime_rounded;
    } else if (time_left_flying < halftime_rounded && time_left_flying > EPSILON) { // Half-time passed
      time_left_z = time_left_flying;
    } else {
      // time_left_z stays 1
    }
  }

  InterpolatePolynomial(solution, X, nbsamples, time_left_xy, state_.x, goal_state.x);
  InterpolatePolynomial(solution, Y, nbsamples, time_left_xy, state_.y, goal_state.y);
  InterpolatePolynomial(solution, Z, nbsamples, time_left_z, state_.z, goal_state.z);
  InterpolatePolynomial(solution, Yaw, nbsamples, time_left_xy, state_.yaw, goal_state.yaw);

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

  if (solution.support_states_vec[0].foot == footType_) {
    FootTrajState.fill(init_state_vec(0));
    FootTrajVel.fill(0.0);
    FootTrajAcc.fill(0.0);

  } else {
    if (solution.support_states_vec[0].phase == DS) {
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
