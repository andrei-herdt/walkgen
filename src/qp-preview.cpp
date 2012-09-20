#include <mpc-walkgen/qp-preview.h>

#include <cmath>

using namespace MPCWalkgen;
using namespace Eigen;

const double QPPreview::EPS_ = 1e-6;

//TODO:change name QPPreview to Preview
QPPreview::QPPreview(Reference *ref, RigidBodySystem *robot, const MPCData *mpc_parameters)
  :robot_(robot)
  ,mpc_parameters_(mpc_parameters)
  ,select_matrices_(mpc_parameters->num_samples_horizon)
  ,rotationMatrix_ (CommonMatrixType::Zero(2*mpc_parameters_->num_samples_horizon, 2*mpc_parameters_->num_samples_horizon))
  ,rotationMatrix2_(CommonMatrixType::Zero(2*mpc_parameters_->num_samples_horizon, 2*mpc_parameters_->num_samples_horizon)) {

  statesolver_ = new StateFSM(ref, mpc_parameters);
}

QPPreview::~QPPreview()
{
  delete statesolver_;
}

void QPPreview::previewSamplingTimes(double current_time, 
                                     double firstSamplingPeriod, MPCSolution &solution) {

  solution.samplingTimes_vec.resize(mpc_parameters_->num_samples_horizon + 1, 0);
  std::fill(solution.samplingTimes_vec.begin(), solution.samplingTimes_vec.end(), 0);
  // As for now, only the first sampling period varies
  solution.samplingTimes_vec[0] = current_time;
  solution.samplingTimes_vec[1] = solution.samplingTimes_vec[0] + firstSamplingPeriod;// mpc_parameters_->QPSamplingPeriod;////// //
  for (int sample = 2; sample < mpc_parameters_->num_samples_horizon + 1; sample++) {
      solution.samplingTimes_vec[sample] += solution.samplingTimes_vec[sample - 1] +
          mpc_parameters_->period_qpsample;
    }

}

void QPPreview::previewSupportStates(double firstSamplingPeriod, MPCSolution &solution){

  const BodyState *foot;
  SupportState &current_support = robot_->current_support();

  // SET CURRENT SUPPORT STATE:
  // --------------------------
  statesolver_->setSupportState(0, solution.samplingTimes_vec, current_support);
  current_support.transitional_ds = false;
  if (current_support.state_changed) {
      if (current_support.foot == LEFT) {
          foot = &robot_->body(LEFT_FOOT)->state();
        } else {
          foot = &robot_->body(RIGHT_FOOT)->state();
        }
      current_support.x = foot->x(0);
      current_support.y = foot->y(0);
      current_support.yaw = foot->yaw(0);
      current_support.start_time = solution.samplingTimes_vec[0];
    }
  solution.support_states_vec.push_back(current_support);

  // PREVIEW SUPPORT STATES:
  // -----------------------
  // initialize the previewed support state before previewing
  SupportState previewed_support = current_support;//TODO: Replace =operator by CopyFrom or give to constructor
  previewed_support.stepNumber = 0;
  for (int sample = 1; sample <= mpc_parameters_->num_samples_horizon; sample++) {
      statesolver_->setSupportState(sample, solution.samplingTimes_vec, previewed_support);
      // special treatment for the first instant of transitionalDS
      previewed_support.transitional_ds = false;
      if (previewed_support.state_changed) {
          if (sample == 1) {// robot is already in ds phase
              if (previewed_support.foot == LEFT) {
                  foot = &robot_->body(LEFT_FOOT)->state();
                } else {
                  foot = &robot_->body(RIGHT_FOOT)->state();
                }
              previewed_support.x = foot->x(0);
              previewed_support.y = foot->y(0);
              previewed_support.yaw = foot->yaw(0);
              previewed_support.start_time = solution.samplingTimes_vec[sample];
              if (current_support.phase == SS && previewed_support.phase == SS) {
                  previewed_support.transitional_ds = true;
                }
            }
          if (/*pi > 1 &&*/ previewed_support.stepNumber > 0) {
              previewed_support.x = 0.0;
              previewed_support.y = 0.0;
            }
        }
      if (sample == 1) {
          previewed_support.previousSamplingPeriod = firstSamplingPeriod;
          previewed_support.sampleWeight = 1;
        } else {
          previewed_support.previousSamplingPeriod = mpc_parameters_->period_qpsample;
          previewed_support.sampleWeight = 1;
        }
      solution.support_states_vec.push_back(previewed_support);
    }

  buildSelectionMatrices(solution);
}

// Fill the two rotation matrices.
//  The indexes not given are supposed to be zero and are not reset
//  to reduce computation time.
void QPPreview::computeRotationMatrix(MPCSolution &solution){
  int N = mpc_parameters_->num_samples_horizon;

  for (int i=0; i<N; ++i) {
      double cosYaw = cos(solution.support_states_vec[i+1].yaw);
      double sinYaw = sin(solution.support_states_vec[i+1].yaw);
      rotationMatrix_(i  ,i  ) =  cosYaw;
      rotationMatrix_(i+N,i  ) = -sinYaw;
      rotationMatrix_(i  ,i+N) =  sinYaw;
      rotationMatrix_(i+N,i+N) =  cosYaw;

      rotationMatrix2_(2*i  ,2*i  ) =  cosYaw;//TODO: Seems to be not used
      rotationMatrix2_(2*i+1,2*i  ) = -sinYaw;
      rotationMatrix2_(2*i  ,2*i+1) =  sinYaw;
      rotationMatrix2_(2*i+1,2*i+1) =  cosYaw;
    }
}

void QPPreview::buildSelectionMatrices(MPCSolution &solution)
{
  assert(select_matrices_.V.rows() == mpc_parameters_->num_samples_horizon);

  const BodyState *left_foot_p = &robot_->body(LEFT_FOOT)->state();
  const BodyState *right_foot_p = &robot_->body(RIGHT_FOOT)->state();

  int num_steps_previewed = solution.support_states_vec.back().stepNumber;
  int num_samples = mpc_parameters_->num_samples_horizon;

  if (select_matrices_.V.cols() != num_steps_previewed){
      select_matrices_.V.resize(num_samples, num_steps_previewed);
      select_matrices_.VT.resize(num_steps_previewed, num_samples);
      select_matrices_.Vf.resize(num_steps_previewed, num_steps_previewed);
      select_matrices_.VcfX.resize(num_steps_previewed);
      select_matrices_.VcfY.resize(num_steps_previewed);
      select_matrices_.m_feet.resize(num_samples, num_steps_previewed);
      select_matrices_.m_feet_trans.resize(num_steps_previewed, num_samples);
    }
  select_matrices_.SetZero();

  std::vector<SupportState>::iterator SS_it = solution.support_states_vec.begin();//points at the cur. sup. st.

  ++SS_it;
  for (int i=0; i < num_samples; i++){
      if (SS_it->stepNumber>0) {
          select_matrices_.V(i, SS_it->stepNumber - 1) = select_matrices_.VT(SS_it->stepNumber-1, i) = 1.0;
          select_matrices_.m_feet(i, SS_it->stepNumber - 1) = select_matrices_.m_feet_trans(SS_it->stepNumber-1, i) = 0.5;
          if (SS_it->stepNumber == 1 && SS_it->state_changed && SS_it->phase == SS) {
              --SS_it;
              select_matrices_.VcfX(0) = SS_it->x;
              select_matrices_.VcfY(0) = SS_it->y;
              ++SS_it;

              select_matrices_.Vf(0,0) = 1.0;
            } else if(SS_it->stepNumber > 1) {
              select_matrices_.Vf(SS_it->stepNumber-1, SS_it->stepNumber-2) = -1.0;
              select_matrices_.Vf(SS_it->stepNumber-1, SS_it->stepNumber-1) = 1.0;
            }
        } else {
          if (SS_it->phase == SS) {
          select_matrices_.VcX(i) = SS_it->x;
          select_matrices_.VcY(i) = SS_it->y;
          select_matrices_.cm_feet_x(i) = SS_it->x;
          select_matrices_.cm_feet_y(i) = SS_it->y;
          } else {
            select_matrices_.cm_feet_x(i) = left_foot_p->x(0) / 2. + right_foot_p->x(0) / 2.;
            select_matrices_.cm_feet_y(i) = left_foot_p->y(0) / 2. + right_foot_p->y(0) / 2.;
          }
          
        }
      ++SS_it;
    }

}
