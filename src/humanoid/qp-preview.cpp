#include "qp-preview.h"

#include <cmath>

using namespace MPCWalkgen;

using namespace Eigen;

const double QPPreview::EPS_ = 1e-6;

//TODO:change name QPPreview to Preview
QPPreview::QPPreview(Reference * velRef, RigidBodySystem * robot, const MPCData * generalData)
  :robot_(robot)
  ,generalData_(generalData)
  ,selectionMatrices_(*generalData)
  ,rotationMatrix_ (Eigen::MatrixXd::Zero(2*generalData_->nbsamples_qp, 2*generalData_->nbsamples_qp))
  ,rotationMatrix2_(Eigen::MatrixXd::Zero(2*generalData_->nbsamples_qp, 2*generalData_->nbsamples_qp)) {

  statesolver_ = new StateFSM(velRef, generalData);
}

QPPreview::~QPPreview()
{
  delete statesolver_;
}

void QPPreview::previewSamplingTimes(double currenttime, 
                                     double firstSamplingPeriod, MPCSolution &solution) {

  solution.samplingTimes_vec.resize(generalData_->nbsamples_qp + 1, 0);
  std::fill(solution.samplingTimes_vec.begin(), solution.samplingTimes_vec.end(), 0);
  // As for now, only the first sampling period varies
  solution.samplingTimes_vec[0] = currenttime;
  solution.samplingTimes_vec[1] = solution.samplingTimes_vec[0] + firstSamplingPeriod;// generalData_->QPSamplingPeriod;////// //
  for (int sample = 2; sample < generalData_->nbsamples_qp + 1; sample++) {
      solution.samplingTimes_vec[sample] += solution.samplingTimes_vec[sample - 1] +
          generalData_->period_qpsample;
    }

}

void QPPreview::previewSupportStates(double firstSamplingPeriod, MPCSolution &solution){

  const BodyState *foot;
  SupportState &currentSupport = robot_->currentSupport();

  // SET CURRENT SUPPORT STATE:
  // --------------------------
  statesolver_->setSupportState(0, solution.samplingTimes_vec, currentSupport);
  currentSupport.inTransitionalDS = false;
  if (currentSupport.stateChanged) {
      if (currentSupport.foot == LEFT) {
          foot = &robot_->body(LEFT_FOOT)->state();
        } else {
          foot = &robot_->body(RIGHT_FOOT)->state();
        }
      currentSupport.x = foot->x(0);
      currentSupport.y = foot->y(0);
      currentSupport.yaw = foot->yaw(0);
      currentSupport.startTime = solution.samplingTimes_vec[0];
    }
  solution.supportStates_vec.push_back(currentSupport);

  // PREVIEW SUPPORT STATES:
  // -----------------------
  // initialize the previewed support state before previewing
  SupportState previewedSupport = currentSupport;
  previewedSupport.stepNumber = 0;
  for (int sample = 1; sample <= generalData_->nbsamples_qp; sample++) {
      statesolver_->setSupportState(sample, solution.samplingTimes_vec, previewedSupport);
      // special treatment for the first instant of transitionalDS
      previewedSupport.inTransitionalDS = false;
      if (previewedSupport.stateChanged) {
          if (sample == 1) {// robot is already in ds phase
              if (previewedSupport.foot == LEFT) {
                  foot = &robot_->body(LEFT_FOOT)->state();
                } else {
                  foot = &robot_->body(RIGHT_FOOT)->state();
                }
              previewedSupport.x = foot->x(0);
              previewedSupport.y = foot->y(0);
              previewedSupport.yaw = foot->yaw(0);
              previewedSupport.startTime = solution.samplingTimes_vec[sample];
              if (currentSupport.phase == SS && previewedSupport.phase == SS) {
                  previewedSupport.inTransitionalDS = true;
                }
            }
          if (/*pi > 1 &&*/ previewedSupport.stepNumber > 0) {
              previewedSupport.x = 0.0;
              previewedSupport.y = 0.0;
            }
        }
      if (sample == 1) {
          previewedSupport.previousSamplingPeriod = firstSamplingPeriod;
          previewedSupport.sampleWeight = 1;
        } else {
          previewedSupport.previousSamplingPeriod = generalData_->period_qpsample;
          previewedSupport.sampleWeight = 1;
        }
      solution.supportStates_vec.push_back(previewedSupport);
    }

  buildSelectionMatrices(solution);
}

// Fill the two rotation matrices.
//  The indexes not given are supposed to be zero and are not reset
//  to reduce computation time.
void QPPreview::computeRotationMatrix(MPCSolution &solution){
  int N = generalData_->nbsamples_qp;

  // check that the elements not on the diagonal are null
  assert(isSparseRotationMatrix(rotationMatrix_) && "The matrix rotationMatrix_ is not 2.2 block diagonal");

  // check that the elements not on the diagonal are null
  assert(isDiagonalRotationMatrix(rotationMatrix2_) && "The matrix rotationMatrix2_ is not 2.2 block diagonal");

  for (int i=0; i<N; ++i) {
      double cosYaw = cos(solution.supportStates_vec[i+1].yaw);
      double sinYaw = sin(solution.supportStates_vec[i+1].yaw);
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

void QPPreview::buildSelectionMatrices(MPCSolution &solution){
  const int &NbPrwSteps = solution.supportStates_vec.back().stepNumber;

  if (selectionMatrices_.V.cols() != NbPrwSteps){
      selectionMatrices_.V.resize(generalData_->nbsamples_qp,NbPrwSteps);
      selectionMatrices_.VT.resize(NbPrwSteps, generalData_->nbsamples_qp);
      selectionMatrices_.VcfX.resize(NbPrwSteps);
      selectionMatrices_.VcfY.resize(NbPrwSteps);
      selectionMatrices_.Vf.resize(NbPrwSteps, NbPrwSteps);
    }

  selectionMatrices_.VcX.fill(0);
  selectionMatrices_.VcY.fill(0);
  selectionMatrices_.V.fill(0);
  selectionMatrices_.VT.fill(0);
  selectionMatrices_.VcfX.fill(0);
  selectionMatrices_.VcfY.fill(0);
  selectionMatrices_.Vf.fill(0);



  std::vector<SupportState>::iterator SS_it;
  SS_it = solution.supportStates_vec.begin();//points at the cur. sup. st.
  ++SS_it;
  for (int i=0; i<generalData_->nbsamples_qp; i++){
      if (SS_it->stepNumber>0){
          selectionMatrices_.V(i,SS_it->stepNumber-1) = selectionMatrices_.VT(SS_it->stepNumber-1,i) = 1.0;
          if (SS_it->stepNumber==1 && SS_it->stateChanged && SS_it->phase == SS) {
              --SS_it;
              selectionMatrices_.VcfX(0) = SS_it->x;
              selectionMatrices_.VcfY(0) = SS_it->y;
              ++SS_it;

              selectionMatrices_.Vf(0,0) = 1.0;
            }else if(SS_it->stepNumber>1){
              selectionMatrices_.Vf(SS_it->stepNumber-1,SS_it->stepNumber-2) = -1.0;
              selectionMatrices_.Vf(SS_it->stepNumber-1,SS_it->stepNumber-1) = 1.0;
            }
        }else{
          selectionMatrices_.VcX(i) = SS_it->x;
          selectionMatrices_.VcY(i) = SS_it->y;
        }
      ++SS_it;
    }

}
