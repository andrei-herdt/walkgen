#include "qp-generator.h"
#include "../common/qp-matrix.h"
#include "../common/qp-vector.h"
#include "../common/tools.h"


#include <iostream>
#include <fstream>


using namespace MPCWalkgen;
using namespace Humanoid;
using namespace Eigen;

QPGenerator::QPGenerator(QPPreview *preview, QPSolver *solver,
                         Reference *velRef, QPPonderation *ponderation,
                         RigidBodySystem *robot, const MPCData *generalData)
                         :preview_(preview)
                         ,solver_(solver)
                         ,robot_(robot)
                         ,velRef_(velRef)
                         ,ponderation_(ponderation)
                         ,generalData_(generalData)
                         ,tmpVec_(1)
                         ,tmpVec2_(1)
                         ,tmpMat_(1,1)
                         ,tmpMat2_(1,1) {
}

QPGenerator::~QPGenerator(){}


void QPGenerator::precomputeObjective(){
  // TODO: Document this function
  int nbUsedPonderations = ponderation_->JerkMin.size(); //
  int size = generalData_->nbFeedbackSamplesStandard();
  Qconst_.resize(size*nbUsedPonderations);
  QconstN_.resize(size*nbUsedPonderations);
  choleskyConst_.resize(size*nbUsedPonderations);
  pconstCoM_.resize(size*nbUsedPonderations);
  pconstVc_.resize(size*nbUsedPonderations);
  pconstRef_.resize(size*nbUsedPonderations);

  int nbsamples = generalData_->nbSamplesQP;
  MatrixXd pondFactor = MatrixXd::Identity(nbsamples,nbsamples);
  MatrixXd G(nbsamples,nbsamples);

  VectorXi order(2*nbsamples);
  for (int i = 0; i < nbsamples; ++i) {
    order(i) = 2*i;
    order(i+nbsamples) = 2*i+1;
  }

  QPMatrix chol(2*nbsamples, 2*nbsamples);
  chol.rowOrder(order);
  chol.colOrder(order);

  for (int i = 0; i < nbUsedPonderations; ++i) {
    for (double s = generalData_->MPCSamplingPeriod;
      s < generalData_->QPSamplingPeriod+EPSILON;
      s += generalData_->MPCSamplingPeriod) {
        int nb = (int)round(s / generalData_->MPCSamplingPeriod)-1;
        nb += i*size;
        robot_->firstSamplingPeriod(s);

        const LinearDynamics &CoPDynamics = robot_->body(COM)->dynamics(copDynamic);
        const LinearDynamics &VelDynamics = robot_->body(COM)->dynamics(velDynamic);

        double firstIterationWeight = s / generalData_->QPSamplingPeriod;
        pondFactor(0, 0) = firstIterationWeight;
        pondFactor(nbsamples-1, nbsamples-1) = 1.05 - firstIterationWeight;


        tmpMat_ = CoPDynamics.UInvT*VelDynamics.UT*pondFactor*VelDynamics.U*CoPDynamics.UInv;
        tmpMat2_= CoPDynamics.UInvT*pondFactor*CoPDynamics.UInv;

        G = ponderation_->instantVelocity[i] * tmpMat_ + ponderation_->JerkMin[i]*tmpMat2_;
        Qconst_[nb] = G;

        QconstN_[nb] = G + ponderation_->CopCentering[i] * MatrixXd::Identity(nbsamples,nbsamples);

        chol.reset();
        chol.addTerm(QconstN_[nb], 0, 0);
        chol.addTerm(QconstN_[nb], nbsamples, nbsamples);

        choleskyConst_[nb] = chol.cholesky();

        pconstCoM_[nb] = VelDynamics.S - VelDynamics.U*CoPDynamics.UInv*CoPDynamics.S;
        pconstCoM_[nb] = CoPDynamics.UInvT*VelDynamics.UT*ponderation_->instantVelocity[i]*pondFactor*pconstCoM_[nb];
        pconstCoM_[nb]-= CoPDynamics.UInvT*ponderation_->JerkMin[i]*pondFactor*CoPDynamics.UInv*CoPDynamics.S;

        pconstVc_[nb]  = CoPDynamics.UInvT*ponderation_->JerkMin[i]*pondFactor*CoPDynamics.UInv;
        pconstVc_[nb] += CoPDynamics.UInvT*VelDynamics.UT*ponderation_->instantVelocity[i]*pondFactor*VelDynamics.U*CoPDynamics.UInv;

        pconstRef_[nb] = -CoPDynamics.UInvT*VelDynamics.UT*ponderation_->instantVelocity[i]*pondFactor;
    }
  }

}

void QPGenerator::BuildProblem(MPCSolution &solution) {

  // DIMENSION OF QP:
  // ----------------
  int nbvars = 2 * generalData_->nbSamplesQP +				// CoM
    2 * solution.supportStates_vec.back().stepNumber;	// Foot placement
  int nbcstr = 5 * solution.supportStates_vec.back().stepNumber;	// Foot placement
  solver_->nbvar(nbvars);
  solver_->nbcstr(nbcstr);

  buildObjective(solution);
  buildConstraints(solution);
  if (solution.useWarmStart)
    computeWarmStart(solution);//TODO: Weird to modify the solution

}

void QPGenerator::buildObjective(const MPCSolution &result) {

  // Choose the precomputed element depending on the nb of "feedback-recomputations" until new qp-sample
  int precomputedMatrixNumber = generalData_->nbFeedbackSamplesLeft(result.supportStates_vec[1].previousSamplingPeriod);
  precomputedMatrixNumber += ponderation_->activePonderation *generalData_->nbFeedbackSamplesStandard();

  const BodyState &CoM = robot_->body(COM)->state();
  const SelectionMatrices &state = preview_->selectionMatrices();
  const MatrixXd &rot = preview_->rotationMatrix();
  const MatrixXd &rot2 = preview_->rotationMatrix2();

  QPMatrix &Q = solver_->matrix(matrixQ);

  int nbsteps_previewed = result.supportStates_vec.back().stepNumber;
  int nbsamples = generalData_->nbSamplesQP;

  //solver_->nbvar(2*nbsamples + 2*nbsteps_previewed);
  //solver_->nbCtr(1); 

  //The LSSOL solver is a bit particular since it uses the cholesky matrix first
  // (the computation of the hessian is hence useless)
  //So if one uses LSSOL, some computation can be avoided.
  bool onlyCholesky = (solver_->useCholesky() == true);

  if (onlyCholesky == false) {
    Q.addTerm(QconstN_[precomputedMatrixNumber], 0, 0);
    Q.addTerm(QconstN_[precomputedMatrixNumber], nbsamples, nbsamples);
  }

  if (nbsteps_previewed>0) {
    tmpMat_ = Qconst_[precomputedMatrixNumber]*state.V;
    Q.addTerm(tmpMat_, 0, 2*nbsamples);
    Q.addTerm(tmpMat_, nbsamples, 2*nbsamples + nbsteps_previewed);


    tmpMat_ = state.VT * Qconst_[precomputedMatrixNumber]*state.V;
    Q.addTerm(tmpMat_, 2 * nbsamples , 2 * nbsamples);
    Q.addTerm(tmpMat_, 2 * nbsamples + nbsteps_previewed, 2*nbsamples + nbsteps_previewed);

    if (onlyCholesky == false){
      tmpMat_ = state.VT*Qconst_[precomputedMatrixNumber];
      Q.addTerm(tmpMat_, 2*nbsamples, 0);
      Q.addTerm(tmpMat_, 2*nbsamples + nbsteps_previewed, nbsamples);

      // rotate the down left block
      MatrixXd dlBlock = Q().block(2*nbsamples, 0, 2*nbsteps_previewed, 2*nbsamples);
      computeMRt(dlBlock, rot2);
      Q().block(2*nbsamples, 0, 2*nbsteps_previewed, 2*nbsamples) = dlBlock;
    }

    // rotate the upper right block
    MatrixXd urBlock = Q().block(0, 2*nbsamples, 2*nbsamples, 2*nbsteps_previewed);
    computeRM(urBlock, rot2);
    Q().block(0, 2*nbsamples, 2*nbsamples, 2*nbsteps_previewed) = urBlock;
  }

  if (onlyCholesky == false){
    MatrixXd Qmat = Q().block(0,0,2*nbsamples,2*nbsamples);
    Qmat = rot2 * Qmat * rot2.transpose();
    Q().block(0, 0, 2 * nbsamples, 2 * nbsamples)=Qmat;
  }

  if ( solver_->useCholesky() == true ){
    // rotate the cholesky matrix
    MatrixXd chol = choleskyConst_[precomputedMatrixNumber];
    rotateCholeskyMatrix(chol, rot2);
    Q.cholesky(chol);
  }

  VectorXd HX(nbsamples),HY(nbsamples),H(2*nbsamples);

  HX = pconstCoM_[precomputedMatrixNumber]*CoM.x;
  HY = pconstCoM_[precomputedMatrixNumber]*CoM.y;

  HX += pconstVc_[precomputedMatrixNumber]*state.VcX;
  HY += pconstVc_[precomputedMatrixNumber]*state.VcY;

  HX += pconstRef_[precomputedMatrixNumber]*velRef_->global.x;
  HY += pconstRef_[precomputedMatrixNumber]*velRef_->global.y;

  if (nbsteps_previewed>0){
    tmpVec_ = state.VT*HX;
    solver_->vector(vectorP).addTerm(tmpVec_, 2*nbsamples);
    tmpVec_ = state.VT*HY;
    solver_->vector(vectorP).addTerm(tmpVec_, 2*nbsamples+nbsteps_previewed);
  }

  H << HX, HY;
  H = rot*H;

  solver_->vector(vectorP).addTerm(H, 0 );

}

void QPGenerator::buildConstraints(const MPCSolution &result){
  int nbsteps_previewed = result.supportStates_vec.back().stepNumber;

  buildConstraintsCOP(result);
  if (nbsteps_previewed>0){
    buildInequalitiesFeet(result);
    buildConstraintsFeet(result);
  }
}

void QPGenerator::computeWarmStart(MPCSolution &result){

  // Initialize:
  // -----------
  int nbSteps = result.supportStates_vec.back().stepNumber;
  int nbStepsMax = generalData_->nbSamplesQP;

  int nbFC = 5;// Number of foot constraints per step TODO: can be read?
  int nbSamples = generalData_->nbSamplesQP;
  result.initialSolution.resize(4*nbSamples + 4*nbSteps);//TODO: 2*nbSamples+2*nbSteps
  //TODO: resize necessary?

  // Preview active set:
  // -------------------
  int size = result.initialConstraints.rows();//TODO: size of what?
  VectorXi initialConstraintTmp = result.initialConstraints;//TODO: Copy not necessary for shifting
  double TimeFactor = result.supportStates_vec[1].sampleWeight;//TODO: TimeFactor? sampleWeight??
  int shiftCtr = 0;
  if (fabs(TimeFactor-1.) < EPSILON) {
    shiftCtr = 1;//if sampleWeight == 1
  }
  if (size >= 2*nbSamples){//TODO: Verification wouldn't be necessary without copying
    // Shift active set by
    result.initialConstraints.segment(0,         nbSamples-1) = initialConstraintTmp.segment(shiftCtr,    nbSamples-1);
    result.initialConstraints.segment(nbSamples, nbSamples-1) = initialConstraintTmp.segment(shiftCtr+nbSamples, nbSamples-1);

    // New final ZMP elements are old final elements
    result.initialConstraints(  nbSamples-1) = initialConstraintTmp(  nbSamples-1);
    result.initialConstraints(2*nbSamples-1) = initialConstraintTmp(2*nbSamples-1);

    // Foot constraints are not shifted
    result.initialConstraints.segment(2*nbSamples          , nbFC*nbSteps)=
      initialConstraintTmp.segment (2*nbSamples          , nbFC*nbSteps);
    result.initialConstraints.segment(2*nbSamples+nbFC*nbSteps, nbFC*(nbStepsMax-nbSteps))=
      initialConstraintTmp.segment (2*nbSamples+nbFC*nbSteps, nbFC*(nbStepsMax-nbSteps));
  } else {
    result.initialConstraints = VectorXi::Zero(2*nbSamples+(4+nbFC)*nbStepsMax);//TODO: Why this value?
  }

  //TODO: Checked until here.

  // Compute feasible initial ZMP and foot positions:
  // ------------------------------------------------
  std::vector<SupportState>::iterator prwSS_it = result.supportStates_vec.begin();
  ++prwSS_it;//Point at the first previewed support state

  SupportState currentSupport = result.supportStates_vec.front();
  // if in transition phase
  if (prwSS_it->stateChanged){
    currentSupport = *prwSS_it;
  }
  int j = 0;

  double shiftx,shifty;
  bool noActiveConstraints;
  for (int i = 0; i<nbSamples; i++){
    // Get COP convex hull for current support
    robot_->convexHull(COPFeasibilityEdges, CoPHull, *prwSS_it, false);

    // Check if the support foot has changed
    if (prwSS_it->stateChanged && prwSS_it->stepNumber>0){

      // Get feet convex hull for current support
      prwSS_it--;
      robot_->convexHull(FootFeasibilityEdges, FootHull,*prwSS_it, false);
      prwSS_it++;

      // Place the foot on active constraints
      shiftx=shifty=0;
      noActiveConstraints=true;
      for(int k=0;k<nbFC;++k){
        if (result.initialConstraints(k+2*nbSamples+j*nbFC)!=0){
          int k2=(k+1)%5; // k(4) = k(0)
          if (result.initialConstraints(k2+2*nbSamples+j*nbFC)!=0){
            shiftx=FootFeasibilityEdges.x(k2);
            shifty=FootFeasibilityEdges.y(k2);
          }else{
            shiftx=(FootFeasibilityEdges.x(k)+FootFeasibilityEdges.x(k2))/2;
            shifty=(FootFeasibilityEdges.y(k)+FootFeasibilityEdges.y(k2))/2;
          }
          noActiveConstraints=false;
          break;
        }
      }
      if (noActiveConstraints){
        shiftx=(FootFeasibilityEdges.x(4)+FootFeasibilityEdges.x(0))/2;
        shifty=(FootFeasibilityEdges.y(4)+FootFeasibilityEdges.y(2))/2;
      }

      currentSupport.x += shiftx;
      currentSupport.y += shifty;

      // Set the new position into initial solution vector
      result.initialSolution(2*nbSamples+j) = currentSupport.x;
      result.initialSolution(2*nbSamples+nbSteps+j) = currentSupport.y;
      ++j;
    }
    // Place the ZMP on active constraints
    shiftx=shifty=0;
    noActiveConstraints=true;
    int k1=-1;
    int k2=-1;
    if (result.initialConstraints(0+i*2)==1){
      if (result.initialConstraints(nbSamples+i*2)==1){
        k2=1;
        noActiveConstraints=false;
      }else if (result.initialConstraints(nbSamples+i*2)==2){
        k2=0;
        noActiveConstraints=false;
      }else if (result.initialConstraints(nbSamples+i*2)==0){
        k1=0;
        k2=1;
        noActiveConstraints=false;
      }
    }else if (result.initialConstraints(0+i*2)==2){
      if (result.initialConstraints(nbSamples+i*2)==1){
        k2=2;
        noActiveConstraints=false;
      }else if (result.initialConstraints(nbSamples+i*2)==2){
        k2=3;
        noActiveConstraints=false;
      }else if (result.initialConstraints(nbSamples+i*2)==0){
        k1=3;
        k2=2;
        noActiveConstraints=false;
      }
    }else if (result.initialConstraints(nbSamples+i*2)==1){
      k1=2;
      k2=1;
      noActiveConstraints=false;
    }else if (result.initialConstraints(nbSamples+i*2)==2){
      k1=0;
      k2=3;
      noActiveConstraints=false;
    }

    if (!noActiveConstraints){
      if (k1!=-1){
        shiftx=(COPFeasibilityEdges.x[k1]+COPFeasibilityEdges.x[k2])/2;
        shifty=(COPFeasibilityEdges.y[k1]+COPFeasibilityEdges.y[k2])/2;
      }else{
        shiftx=COPFeasibilityEdges.x[k2];
        shifty=COPFeasibilityEdges.y[k2];
      }
    }

    result.initialSolution(i) = shiftx;
    result.initialSolution(nbSamples+i) = shifty;
    ++prwSS_it;

  }

}

void QPGenerator::computeReferenceVector(const MPCSolution &result){

  if (velRef_->global.x.rows()!=generalData_->nbSamplesQP){
    velRef_->global.x.resize(generalData_->nbSamplesQP);
    velRef_->global.y.resize(generalData_->nbSamplesQP);
  }

  double YawTrunk;
  for (int i=0;i<generalData_->nbSamplesQP;++i){
    YawTrunk = result.supportStates_vec[i+1].yaw;
    velRef_->global.x(i) = velRef_->local.x(i)*cos(YawTrunk)-velRef_->local.y(i)*sin(YawTrunk);
    velRef_->global.y(i) = velRef_->local.x(i)*sin(YawTrunk)+velRef_->local.y(i)*cos(YawTrunk);
  }

}

void QPGenerator::convertCopToJerk(MPCSolution &result){
  int nbsamples = generalData_->nbSamplesQP;

  const SelectionMatrices &State = preview_->selectionMatrices();
  const MatrixXd &rot = preview_->rotationMatrix();
  const BodyState &CoM = robot_->body(COM)->state();
  const LinearDynamics &CoP = robot_->body(COM)->dynamics(copDynamic);
  // int nbSteps =  result.supportState_vec.back().stepNumber;

  VectorXd sx = result.qpSolution.segment(0, 2);
  VectorXd sy = result.qpSolution.segment(nbsamples, 2);

  // Vpx(0,0) and Vpy(0,0) always equal to 0 because the first iteration always on the current foot and never on previewed steps
  /*
  const VectorXd px = result.solution.segment(2*nbsamples,         nbSteps);
  const VectorXd py = result.solution.segment(2*nbsamples+nbSteps, nbSteps);
  VectorXd Vpx;
  VectorXd Vpy;
  if (nbSteps>0){
  Vpx =State.V*px;
  Vpy =State.V*py;
  }else{
  Vpx=VectorXd::Zero(nbsamples);
  Vpy=VectorXd::Zero(nbsamples);
  }
  */


  double zx;
  zx =(rot.block(0,0,1,2)*sx -rot.block(0,nbsamples,1,2)*sy)(0,0);
  zx+=/*Vpx(0,0)+*/State.VcX(0,0);

  double zy;
  zy =(rot.block(nbsamples,nbsamples,1,2)*sy -rot.block(nbsamples,0,1,2)*sx)(0,0);
  zy+=/*Vpy(0,0)+*/State.VcY(0,0);

  double X;
  double Y;
  zx -= (CoP.S.block(0,0,1,3) *CoM.x)(0,0);
  X  =  CoP.UInv(0,0) *zx;
  zy -= (CoP.S.block(0,0,1,3) *CoM.y)(0,0) ;
  Y  =  CoP.UInv(0,0) *zy;

  result.qpSolution.segment(0, nbsamples).fill(X);
  result.qpSolution.segment(nbsamples, nbsamples).fill(Y);

}

void QPGenerator::buildInequalitiesFeet(const MPCSolution &result){

  int nbIneq = 5;
  int nbSteps = result.supportStates_vec.back().stepNumber;

  feetInequalities_.resize(nbIneq*nbSteps , nbSteps);

  std::vector<SupportState>::const_iterator prwSS_it = result.supportStates_vec.begin();
  prwSS_it++;//Point at the first previewed instant
  for( int i = 0; i < generalData_->nbSamplesQP; ++i ){
    //foot positioning constraints
    if( prwSS_it->stateChanged && prwSS_it->stepNumber > 0 && prwSS_it->phase != DS){

      prwSS_it--;//Foot polygons are defined with respect to the supporting foot
      robot_->convexHull(hull, FootHull, *prwSS_it);
      prwSS_it++;

      int stepNumber = (prwSS_it->stepNumber-1);

      feetInequalities_.DX.block( stepNumber * nbIneq, stepNumber, nbIneq, 1) = hull.A.segment(0, nbIneq);
      feetInequalities_.DY.block( stepNumber * nbIneq, stepNumber, nbIneq, 1) = hull.B.segment(0, nbIneq);
      feetInequalities_.Dc.segment(stepNumber * nbIneq, nbIneq) = hull.D.segment(0, nbIneq);
    }
    prwSS_it++;
  }

}

void QPGenerator::buildConstraintsFeet(const MPCSolution &result){

  int nbsteps_previewed = result.supportStates_vec.back().stepNumber;
  const SelectionMatrices &State = preview_->selectionMatrices();
  int nbsamples = generalData_->nbSamplesQP;

  tmpMat_.noalias() = feetInequalities_.DX * State.Vf;
  solver_->matrix(matrixA).addTerm(tmpMat_,  0,  2 * nbsamples);

  tmpMat_.noalias() = feetInequalities_.DY * State.Vf;
  solver_->matrix(matrixA).addTerm(tmpMat_,  0, 2 * nbsamples + nbsteps_previewed);

  solver_->vector(vectorBL).addTerm(feetInequalities_.Dc, 0);

  tmpVec_ =  feetInequalities_.DX * State.VcfX;
  tmpVec_ += feetInequalities_.DY * State.VcfY;
  solver_->vector(vectorBL).addTerm(tmpVec_,  0);

  solver_->vector(vectorBU)().segment(0, tmpVec_.size()).fill(10e10);
}

void QPGenerator::buildConstraintsCOP(const MPCSolution &result) {

  int nbsamples = generalData_->nbSamplesQP;
  std::vector<SupportState>::const_iterator prwSS_it = result.supportStates_vec.begin();

  robot_->convexHull(hull, CoPHull, *prwSS_it, false, false);

  int nbsteps_previewed = result.supportStates_vec.back().stepNumber;
  int size = 2 * nbsamples + 2 * nbsteps_previewed;
  tmpVec_.resize(size);
  tmpVec2_.resize(size);

  ++prwSS_it;//Point at the first previewed instant
  for(int i = 0; i < nbsamples; ++i) {
    if( prwSS_it->stateChanged ) {
      robot_->convexHull(hull, CoPHull, *prwSS_it, false, false);
    }
    tmpVec_(i)  = std::min(hull.x(0), hull.x(3));
    tmpVec2_(i) = std::max(hull.x(0), hull.x(3));

    tmpVec_(nbsamples + i) = std::min(hull.y(0),hull.y(1));
    tmpVec2_(nbsamples + i)= std::max(hull.y(0),hull.y(1));
    ++prwSS_it;
  }

  tmpVec_.segment(2 * nbsamples, 2 * nbsteps_previewed).fill(-10e10);
  tmpVec2_.segment(2 * nbsamples, 2 * nbsteps_previewed).fill(10e10);

  int first_row = 0;
  solver_->vector(vectorXL).addTerm(tmpVec_, first_row);
  solver_->vector(vectorXU).addTerm(tmpVec2_, first_row);


}



void QPGenerator::display(const MPCSolution &result, const std::string &filename) const
{
  int nbsamples = generalData_->nbSamplesQP;

  const SelectionMatrices &State = preview_->selectionMatrices();
  const MatrixXd &rot = preview_->rotationMatrix();
  const BodyState &CoM = robot_->body(COM)->state();
  const LinearDynamics &CoP = robot_->body(COM)->dynamics(copDynamic);
  int nbSteps =  result.supportStates_vec.back().stepNumber;

  const VectorXd &sx = result.qpSolution.segment(0, nbsamples);
  const VectorXd &sy = result.qpSolution.segment(nbsamples, nbsamples);

  const VectorXd px = result.qpSolution.segment(2*nbsamples,         nbSteps);
  const VectorXd py = result.qpSolution.segment(2*nbsamples+nbSteps, nbSteps);

  VectorXd Vpx;
  VectorXd Vpy;
  if (nbSteps > 0){
    Vpx =State.V*px;
    Vpy =State.V*py;
  } else {
    Vpx=VectorXd::Zero(nbsamples);
    Vpy=VectorXd::Zero(nbsamples);
  }

  VectorXd zx(nbsamples);
  zx = rot.block(0,0,nbsamples,nbsamples)*sx -rot.block(0,nbsamples,nbsamples,nbsamples)*sy;
  zx += Vpx+State.VcX;

  VectorXd zy(nbsamples);
  zy = rot.block(nbsamples,nbsamples,nbsamples,nbsamples)*sy -rot.block(nbsamples,0,nbsamples,nbsamples)*sx;
  zy += Vpy+State.VcY;

  VectorXd X;
  VectorXd Y;
  zx -= CoP.S *CoM.x;
  X  =  CoP.UInv *zx;

  zy -= CoP.S *CoM.y ;
  Y  =  CoP.UInv *zy;



  std::ofstream data(filename.c_str());
  if (!data)
  {
    std::cerr << "Unable to open " << filename << std::endl;
    return;
  }

  int nbSamples = generalData_->nbSamplesQP;

  VectorXd ZX(nbSamples);
  VectorXd ZY(nbSamples);
  VectorXd CX(nbSamples);
  VectorXd CY(nbSamples);



  const LinearDynamics &CoMPos = robot_->body(COM)->dynamics(posDynamic);


  // Compute previewed ZMP
  ZX=CoP.S *CoM.x + CoP.U*X;
  ZY=CoP.S *CoM.y + CoP.U*Y;

  CX = CoMPos.S*CoM.x + CoMPos.U*X;
  CY = CoMPos.S*CoM.y + CoMPos.U*Y;

  //display previewed ZMP

  for(int i=0;i<generalData_->nbSamplesQP;++i){
    data << "TRAJ\t1\t\t0\t1\t1\t\t" << ZX(i) << "\t" << ZY(i) << "\t0\n";
  }

  //display previewed COM

  for(int i=0;i<generalData_->nbSamplesQP;++i){
    data << "TRAJ\t2\t\t1\t0\t0\t\t" << CX(i) << "\t" << CY(i) << "\t0\n";
  }



  SupportState currentSupport = result.supportStates_vec.front();
  std::vector<SupportState>::const_iterator prwSS_it = result.supportStates_vec.begin();


  int j = 0, b = 0;


  double Xfoot, Yfoot;

  //display current COP constraint
  ConvexHull COPFeasibilityEdges = robot_->convexHull(CoPHull, *prwSS_it, false);

  for (int k=0; k<4; ++k) {
    data << "BOUND\t-1\t\t0.1\t0.8\t0.1\t\t" <<
      COPFeasibilityEdges.x[k]+currentSupport.x << "\t" <<
      COPFeasibilityEdges.y[k]+currentSupport.y << "\t0\n";
  }
  data << "BOUND\t-1\t\t0.1\t0.8\t0.1\t\t" <<
    COPFeasibilityEdges.x[0]+currentSupport.x << "\t" <<
    COPFeasibilityEdges.y[0]+currentSupport.y << "\t0\n";

  //display current feet positions
  data << "POINT\t-1\t\t0.1\t0.8\t0.1\t\t" <<
    currentSupport.x << "\t" <<
    currentSupport.y << "\t0\n";
  ++b;

  prwSS_it++;

  for(int i=0; i<generalData_->nbSamplesQP; i++)
  {

    //display constraints
    if (prwSS_it->stateChanged){

      COPFeasibilityEdges = robot_->convexHull(CoPHull, *prwSS_it, false);
      ConvexHull FootFeasibilityEdges = robot_->convexHull(FootHull, *prwSS_it, false);

      if (prwSS_it->stepNumber == 0) {
        Xfoot=result.supportStates_vec[0].x;
        Yfoot=result.supportStates_vec[0].y;
      } else {

        Xfoot=result.qpSolution(2*generalData_->nbSamplesQP+j);
        Yfoot=result.qpSolution(2*generalData_->nbSamplesQP+nbSteps+j);
        if (j+1<nbSteps) {
          j++;
        }
      }

      if (prwSS_it->inTransitionalDS){
        Xfoot=result.supportStates_vec[1].x;
        Yfoot=result.supportStates_vec[1].y;
      }

      //display COP constraints
      for (int k=0; k<4; ++k) {
        data << "BOUND\t" << b << "\t\t0.5\t0.5\t0.5\t\t" <<
          COPFeasibilityEdges.x[k]+Xfoot << "\t" <<
          COPFeasibilityEdges.y[k]+Yfoot << "\t0\n";
      }

      //display feet constraints
      for (int k=0; k<5; ++k) {
        data << "BOUND\t" << b+10000 << "\t\t0.5\t0.5\t0.5\t\t" <<
          FootFeasibilityEdges.x[k]+Xfoot << "\t" <<
          FootFeasibilityEdges.y[k]+Yfoot << "\t0\n";
      }

      //display feet positions
      data << "POINT\t" << b+100000 << "\t\t1\t0\t0\t\t" <<
        Xfoot << "\t" <<
        Yfoot << "\t0\n";
      ++b;
    }
    prwSS_it++;

  }
  data.close();
}

