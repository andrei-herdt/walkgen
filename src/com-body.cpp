#include <mpc-walkgen/com-body.h>
#include <mpc-walkgen/tools.h>

using namespace MPCWalkgen;

using namespace Eigen;


CoMBody::CoMBody() : RigidBody() {}

CoMBody::~CoMBody() {}

void CoMBody::Interpolate(MPCSolution &solution, double currentTime, const Reference &velRef){
  // Actuator sampling rate:
  // -----------------------
  // Position:
  interpolation_.Interpolate(solution.com_act.pos.x_vec, dynamics_act().pos, 
    state_.x, solution.com_prw.jerk.x_vec[0]);
  interpolation_.Interpolate(solution.com_act.pos.y_vec, dynamics_act().pos, 
    state_.y, solution.com_prw.jerk.y_vec[0]);

  // Velocity:
  interpolation_.Interpolate(solution.com_act.vel.x_vec, dynamics_act().vel, 
    state_.x, solution.com_prw.jerk.x_vec[0]);
  interpolation_.Interpolate(solution.com_act.vel.y_vec, dynamics_act().vel, 
    state_.y, solution.com_prw.jerk.y_vec[0]);

  // Acceleration:
  interpolation_.Interpolate(solution.com_act.acc.x_vec, dynamics_act().acc, 
    state_.x, solution.com_prw.jerk.x_vec[0]);
  interpolation_.Interpolate(solution.com_act.acc.y_vec, dynamics_act().acc, 
    state_.y, solution.com_prw.jerk.y_vec[0]);

  // CoP:
  interpolation_.Interpolate(solution.cop_act.pos.x_vec, dynamics_act().cop, 
    state_.x, solution.com_prw.jerk.x_vec[0]);
  interpolation_.Interpolate(solution.cop_act.pos.y_vec, dynamics_act().cop, 
    state_.y, solution.com_prw.jerk.y_vec[0]);

  // QP sampling rate:
  // -----------------
  // Position:
  if (mpc_parameters_->interpolate_whole_horizon) {
      interpolation_.Interpolate(solution.com_prw.pos.x_vec, dynamics_qp().pos,
      state_.x, solution.com_prw.jerk.x_vec);
      interpolation_.Interpolate(solution.com_prw.pos.y_vec, dynamics_qp().pos,
      state_.y, solution.com_prw.jerk.y_vec);
      
      // Velocity:
      interpolation_.Interpolate(solution.com_prw.vel.x_vec, dynamics_qp().vel,
      state_.x, solution.com_prw.jerk.x_vec);
      interpolation_.Interpolate(solution.com_prw.vel.y_vec, dynamics_qp().vel,
      state_.y, solution.com_prw.jerk.y_vec);
      
      // Acceleration:
      interpolation_.Interpolate(solution.com_prw.acc.x_vec, dynamics_qp().acc,
      state_.x, solution.com_prw.jerk.x_vec);
      interpolation_.Interpolate(solution.com_prw.acc.y_vec, dynamics_qp().acc,
      state_.y, solution.com_prw.jerk.y_vec);
  }

  interpolateTrunkOrientation(solution, currentTime, velRef);
}

void CoMBody::interpolateTrunkOrientation(MPCSolution &solution,
                                          double /*currentTime*/, const Reference &velRef) 
{
  double T = mpc_parameters_->nbqpsamples_step * mpc_parameters_->period_qpsample;

  int nbSampling = mpc_parameters_->num_samples_act();

  if (solution.state_vec[0].trunkYaw_.rows() != nbSampling){
    for (int i = 0; i < 3; ++i) {
      solution.state_vec[i].trunkYaw_.resize(nbSampling);
    }
  }

  double orientation0 = solution.supportTrunkOrientations_vec[0];
  double orientation1;

  int size = solution.supportTrunkOrientations_vec.size();
  if (size >= 2) {
    orientation1 = solution.supportTrunkOrientations_vec[1];
  } else {
    orientation1 = orientation0;
  }

  Vector3d nextTrunkState(orientation1, -velRef.local.yaw(0), 0);

  Eigen::Matrix<double,6,1> factor;
  interpolation_.computePolynomialNormalisedFactors(factor, state().yaw, nextTrunkState, T);
  for (int i=0; i < nbSampling; ++i) {
    double ti = (i + 1) * mpc_parameters_->period_actsample;

    solution.state_vec[0].trunkYaw_(i) = p(factor, ti/T);
    solution.state_vec[1].trunkYaw_(i) = dp(factor, ti/T)/T;
    solution.state_vec[2].trunkYaw_(i) = ddp(factor, ti/T)/(T*T);
  }


}
