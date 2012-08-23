#include <mpc-walkgen/qp-generator.h>
#include <mpc-walkgen/qp-matrix.h>
#include <mpc-walkgen/qp-vector.h>
#include <mpc-walkgen/tools.h>

using namespace MPCWalkgen;
using namespace Eigen;

QPGenerator::QPGenerator(QPPreview *preview, QPSolver *solver,
                         Reference *velRef, QPPonderation *ponderation,
                         RigidBodySystem *robot, const MPCData *mpc_parameters)
                         :preview_(preview)
                         ,solver_(solver)
                         ,robot_(robot)
                         ,velRef_(velRef)
                         ,ponderation_(ponderation)
                         ,mpc_parameters_(mpc_parameters)
                         ,tmp_vec_(1)
                         ,tmp_vec2_(1)
                         ,tmp_mat_(1,1)
                         ,tmp_mat2_(1,1) {
}

QPGenerator::~QPGenerator(){}


void QPGenerator::precomputeObjective(){
  // TODO: Document this function
  int nbUsedPonderations = ponderation_->JerkMin.size(); //
  int size = mpc_parameters_->nbFeedbackSamplesStandard();
  Qconst_.resize(size*nbUsedPonderations);
  QconstN_.resize(size*nbUsedPonderations);
  choleskyConst_.resize(size*nbUsedPonderations);
  pconstCoM_.resize(size*nbUsedPonderations);
  pconstVc_.resize(size*nbUsedPonderations);
  pconstRef_.resize(size*nbUsedPonderations);

  int nbsamples = mpc_parameters_->nbsamples_qp;
  //MatrixXd pondFactor = MatrixXd::Identity(nbsamples,nbsamples);
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
    for (double period_first = mpc_parameters_->period_mpcsample;
      period_first < mpc_parameters_->period_qpsample+EPSILON;
      period_first += mpc_parameters_->period_mpcsample) {
        int nb = (int)round(period_first / mpc_parameters_->period_mpcsample)-1;
        nb += i*size;
        robot_->setSelectionNumber(period_first);
        // TODO: Get access to entire vector and do all operations here
        const LinearDynamicsMatrices &CoPDynamics = robot_->body(COM)->dynamics_qp().cop;
        const LinearDynamicsMatrices &VelDynamics = robot_->body(COM)->dynamics_qp().vel;

        //double firstIterationWeight = period_first / mpc_parameters_->period_qpsample;

        tmp_mat_.noalias() = CoPDynamics.UInvT*VelDynamics.UT/**pondFactor*/*VelDynamics.U*CoPDynamics.UInv;
        tmp_mat2_.noalias() = CoPDynamics.UInvT/**pondFactor*/*CoPDynamics.UInv;

        G = ponderation_->instantVelocity[i] * tmp_mat_ + ponderation_->JerkMin[i]*tmp_mat2_;
        Qconst_[nb] = G;

        QconstN_[nb] = G + ponderation_->CopCentering[i] * MatrixXd::Identity(nbsamples,nbsamples);

        chol.reset();
        chol.addTerm(QconstN_[nb], 0, 0);
        chol.addTerm(QconstN_[nb], nbsamples, nbsamples);

        choleskyConst_[nb] = chol.cholesky();

        pconstCoM_[nb] = VelDynamics.S - VelDynamics.U*CoPDynamics.UInv*CoPDynamics.S;
        pconstCoM_[nb] = CoPDynamics.UInvT*VelDynamics.UT*ponderation_->instantVelocity[i]/**pondFactor*/*pconstCoM_[nb];
        pconstCoM_[nb]-= CoPDynamics.UInvT*ponderation_->JerkMin[i]/**pondFactor*/*CoPDynamics.UInv*CoPDynamics.S;

        pconstVc_[nb]  = CoPDynamics.UInvT*ponderation_->JerkMin[i]/**pondFactor*/*CoPDynamics.UInv;
        pconstVc_[nb] += CoPDynamics.UInvT*VelDynamics.UT*ponderation_->instantVelocity[i]/**pondFactor*/*VelDynamics.U*CoPDynamics.UInv;

        pconstRef_[nb] = -CoPDynamics.UInvT*VelDynamics.UT*ponderation_->instantVelocity[i]/**pondFactor*/;
    }
  }

}

void QPGenerator::BuildProblem(MPCSolution &solution) {

  // DIMENSION OF QP:
  // ----------------
  int nbvars = 2 * mpc_parameters_->nbsamples_qp +				// CoM
    2 * solution.support_states_vec.back().stepNumber;	// Foot placement
  int nbcstr = 5 * solution.support_states_vec.back().stepNumber;	// Foot placement
  solver_->nbvar(nbvars);
  solver_->nbcstr(nbcstr);

  buildObjective(solution);
  buildConstraints(solution);
  if (mpc_parameters_->warmstart)
    computeWarmStart(solution);//TODO: Weird to modify the solution

}

void QPGenerator::buildObjective(const MPCSolution &solution) {

  // Choose the precomputed element depending on the nb of "feedback-recomputations" until new qp-sample
  int precomputedMatrixNumber = mpc_parameters_->nbFeedbackSamplesLeft(solution.support_states_vec[1].previousSamplingPeriod);
  precomputedMatrixNumber += ponderation_->activePonderation * mpc_parameters_->nbFeedbackSamplesStandard();

  const BodyState &CoM = robot_->body(COM)->state();
  const SelectionMatrices &state = preview_->selectionMatrices();
  const MatrixXd &rot = preview_->rotationMatrix();
  const MatrixXd &rot2 = preview_->rotationMatrix2();

  QPMatrix &Q = solver_->matrix(matrixQ);

  int nbsteps_previewed = solution.support_states_vec.back().stepNumber;
  int nbsamples = mpc_parameters_->nbsamples_qp;

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
    tmp_mat_.noalias() = Qconst_[precomputedMatrixNumber]*state.V;
    Q.addTerm(tmp_mat_, 0, 2*nbsamples);
    Q.addTerm(tmp_mat_, nbsamples, 2*nbsamples + nbsteps_previewed);


    tmp_mat_.noalias() = state.VT * Qconst_[precomputedMatrixNumber]*state.V;
    Q.addTerm(tmp_mat_, 2 * nbsamples , 2 * nbsamples);
    Q.addTerm(tmp_mat_, 2 * nbsamples + nbsteps_previewed, 2*nbsamples + nbsteps_previewed);

    if (onlyCholesky == false){
      tmp_mat_.noalias() = state.VT*Qconst_[precomputedMatrixNumber];
      Q.addTerm(tmp_mat_, 2*nbsamples, 0);
      Q.addTerm(tmp_mat_, 2*nbsamples + nbsteps_previewed, nbsamples);

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
    tmp_vec_.noalias() = state.VT * HX;
    solver_->vector(vectorP).addTerm(tmp_vec_, 2 * nbsamples);
    tmp_vec_.noalias() = state.VT * HY;
    solver_->vector(vectorP).addTerm(tmp_vec_, 2 * nbsamples + nbsteps_previewed);
  }

  H << HX, HY;
  H = rot * H;

  solver_->vector(vectorP).addTerm(H, 0 );

}

void QPGenerator::buildConstraints(const MPCSolution &solution){
  int nbsteps_previewed = solution.support_states_vec.back().stepNumber;

  buildConstraintsCOP(solution);
  if (nbsteps_previewed>0){
    buildInequalitiesFeet(solution);
    buildConstraintsFeet(solution);
  }
}

void QPGenerator::computeWarmStart(MPCSolution &solution){

  // Initialize:
  // -----------
  int nbSteps = solution.support_states_vec.back().stepNumber;
  int nbStepsMax = mpc_parameters_->nbsamples_qp;

  int nbFC = 5;// Number of foot constraints per step TODO: can be read?
  int nbsamples = mpc_parameters_->nbsamples_qp;
  solution.initialSolution.resize(4*nbsamples + 4*nbSteps);//TODO: 2*nbsamples+2*nbSteps
  //TODO: resize necessary?

  // Preview active set:
  // -------------------
  int size = solution.initialConstraints.rows();//TODO: size of what?
  VectorXi initialConstraintTmp = solution.initialConstraints;//TODO: Copy not necessary for shifting
  double TimeFactor = solution.support_states_vec[1].sampleWeight;//TODO: TimeFactor? sampleWeight??
  int shiftCtr = 0;
  if (fabs(TimeFactor-1.) < EPSILON) {
    shiftCtr = 1;//if sampleWeight == 1
  }
  if (size >= 2*nbsamples){//TODO: Verification wouldn't be necessary without copying
    // Shift active set by
    solution.initialConstraints.segment(0,         nbsamples-1) = initialConstraintTmp.segment(shiftCtr,    nbsamples-1);
    solution.initialConstraints.segment(nbsamples, nbsamples-1) = initialConstraintTmp.segment(shiftCtr+nbsamples, nbsamples-1);

    // New final ZMP elements are old final elements
    solution.initialConstraints(  nbsamples-1) = initialConstraintTmp(  nbsamples-1);
    solution.initialConstraints(2*nbsamples-1) = initialConstraintTmp(2*nbsamples-1);

    // Foot constraints are not shifted
    solution.initialConstraints.segment(2*nbsamples          , nbFC*nbSteps)=
      initialConstraintTmp.segment (2*nbsamples          , nbFC*nbSteps);
    solution.initialConstraints.segment(2*nbsamples+nbFC*nbSteps, nbFC*(nbStepsMax-nbSteps))=
      initialConstraintTmp.segment (2*nbsamples+nbFC*nbSteps, nbFC*(nbStepsMax-nbSteps));
  } else {
    solution.initialConstraints = VectorXi::Zero(2*nbsamples+(4+nbFC)*nbStepsMax);//TODO: Why this value?
  }

  //TODO: Checked until here.

  // Compute feasible initial ZMP and foot positions:
  // ------------------------------------------------
  std::vector<SupportState>::iterator prwSS_it = solution.support_states_vec.begin();
  ++prwSS_it;//Point at the first previewed support state

  SupportState current_support = solution.support_states_vec.front();
  // if in transition phase
  if (prwSS_it->state_changed){
    current_support = *prwSS_it;
  }
  int j = 0;

  double shiftx,shifty;
  bool noActiveConstraints;
  for (int i = 0; i<nbsamples; i++){
    // Get COP convex hull for current support
    robot_->convexHull(COPFeasibilityEdges, CoPHull, *prwSS_it, false);

    // Check if the support foot has changed
    if (prwSS_it->state_changed && prwSS_it->stepNumber>0){

      // Get feet convex hull for current support
      prwSS_it--;
      robot_->convexHull(FootFeasibilityEdges, FootHull,*prwSS_it, false);
      prwSS_it++;

      // Place the foot on active constraints
      shiftx=shifty=0;
      noActiveConstraints=true;
      for(int k=0;k<nbFC;++k){
        if (solution.initialConstraints(k+2*nbsamples+j*nbFC)!=0){
          int k2=(k+1)%5; // k(4) = k(0)
          if (solution.initialConstraints(k2+2*nbsamples+j*nbFC)!=0){
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

      current_support.x += shiftx;
      current_support.y += shifty;

      // Set the new position into initial solution vector
      solution.initialSolution(2*nbsamples+j) = current_support.x;
      solution.initialSolution(2*nbsamples+nbSteps+j) = current_support.y;
      ++j;
    }
    // Place the ZMP on active constraints
    shiftx=shifty=0;
    noActiveConstraints=true;
    int k1=-1;
    int k2=-1;
    if (solution.initialConstraints(0+i*2)==1){
      if (solution.initialConstraints(nbsamples+i*2)==1){
        k2=1;
        noActiveConstraints=false;
      }else if (solution.initialConstraints(nbsamples+i*2)==2){
        k2=0;
        noActiveConstraints=false;
      }else if (solution.initialConstraints(nbsamples+i*2)==0){
        k1=0;
        k2=1;
        noActiveConstraints=false;
      }
    }else if (solution.initialConstraints(0+i*2)==2){
      if (solution.initialConstraints(nbsamples+i*2)==1){
        k2=2;
        noActiveConstraints=false;
      }else if (solution.initialConstraints(nbsamples+i*2)==2){
        k2=3;
        noActiveConstraints=false;
      }else if (solution.initialConstraints(nbsamples+i*2)==0){
        k1=3;
        k2=2;
        noActiveConstraints=false;
      }
    }else if (solution.initialConstraints(nbsamples+i*2)==1){
      k1=2;
      k2=1;
      noActiveConstraints=false;
    }else if (solution.initialConstraints(nbsamples+i*2)==2){
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

    solution.initialSolution(i) = shiftx;
    solution.initialSolution(nbsamples+i) = shifty;
    ++prwSS_it;

  }

}

void QPGenerator::computeReferenceVector(const MPCSolution &solution){

  if (velRef_->global.x.rows() != mpc_parameters_->nbsamples_qp){
    velRef_->global.x.resize(mpc_parameters_->nbsamples_qp);
    velRef_->global.y.resize(mpc_parameters_->nbsamples_qp);
  }

  double YawTrunk;
  for (int i=0;i<mpc_parameters_->nbsamples_qp;++i){
    YawTrunk = solution.support_states_vec[i+1].yaw;
    velRef_->global.x(i) = velRef_->local.x(i)*cos(YawTrunk)-velRef_->local.y(i)*sin(YawTrunk);
    velRef_->global.y(i) = velRef_->local.x(i)*sin(YawTrunk)+velRef_->local.y(i)*cos(YawTrunk);
  }

}

void QPGenerator::ConvertCopToJerk(MPCSolution &solution){
  int nbsamples = mpc_parameters_->nbsamples_qp;

  const SelectionMatrices &select = preview_->selectionMatrices();
  const MatrixXd &rot_mat = preview_->rotationMatrix();
  const BodyState &CoM = robot_->body(COM)->state();

  VectorXd sol_x_vec = solution.qpSolution.segment(0, nbsamples);//TODO: Solution should not be overwritten
  VectorXd sol_y_vec = solution.qpSolution.segment(nbsamples, nbsamples);
  int nbsteps = solution.support_states_vec.back().stepNumber;
  const VectorXd feet_x_vec = solution.qpSolution.segment(2 * nbsamples, nbsteps);
  const VectorXd feet_y_vec = solution.qpSolution.segment(2 * nbsamples + nbsteps, nbsteps);


  int nbsamples_interp = 1;
  if (mpc_parameters_->interpolate_preview == true) {
    nbsamples_interp = nbsamples;
  }

  VectorXd &cop_x_vec = solution.cop_prw.pos.x_vec;
  VectorXd &cop_y_vec = solution.cop_prw.pos.y_vec;//TODO: Resize OK?
  cop_x_vec.setZero(nbsamples);
  cop_y_vec.setZero(nbsamples);
  for (int s = 0; s < nbsamples_interp; ++s) {
    // Rotate: Rx*x - Ry*y;
    cop_x_vec(s) = 
      rot_mat(s, s) * sol_x_vec(s) - rot_mat(s, nbsamples + s) * sol_y_vec(s);
    cop_y_vec(s) = 
       rot_mat(nbsamples + s, nbsamples + s) * sol_y_vec(s) - rot_mat(nbsamples + s, s) * sol_x_vec(s);
  }
  // Add to global coordinates of the feet
  cop_x_vec += select.VcX;
  cop_x_vec += select.V * feet_x_vec;
  cop_y_vec += select.VcY; 
  cop_y_vec += select.V * feet_y_vec;

  // Transform to jerk vectors
  const LinearDynamicsMatrices &copdyn = robot_->body(COM)->dynamics_qp().cop;
  VectorXd &jerk_x_vec = solution.com_prw.jerk.x_vec;
  jerk_x_vec.setZero(nbsamples); 
  VectorXd &jerk_y_vec = solution.com_prw.jerk.y_vec;
  jerk_y_vec.setZero(nbsamples); 
  tmp_vec_ = cop_x_vec - copdyn.S * CoM.x;
  jerk_x_vec.noalias() = copdyn.UInv * tmp_vec_;
  tmp_vec_ = cop_y_vec - copdyn.S * CoM.y;
  jerk_y_vec.noalias() = copdyn.UInv * tmp_vec_;

  //TODO: We don't need the rest
  // Vpx(0,0) and Vpy(0,0) always equal to 0 because the first iteration always on the current foot and never on previewed steps
  /*
  const VectorXd px = solution.solution.segment(2*nbsamples,         nbSteps);
  const VectorXd py = solution.solution.segment(2*nbsamples+nbSteps, nbSteps);
  VectorXd Vpx;
  VectorXd Vpy;
  if (nbSteps>0){
  Vpx =select.V*px;
  Vpy =select.V*py;
  }else{
  Vpx=VectorXd::Zero(nbsamples);
  Vpy=VectorXd::Zero(nbsamples);
  }
  */

  // Compute ZMP wrt the inertial frame:
  // -----------------------------------
  // TODO: The jerk is computed only for the first instant and copied for the whole preview period!!
  
  double zx;
  zx =(rot_mat.block(0,0,1,2) * sol_x_vec.segment(0, 2) - rot_mat.block(0,nbsamples,1,2)*sol_y_vec.segment(0, 2))(0,0);
  zx+=/*Vpx(0,0)+*/select.VcX(0,0);

  double zy;
  zy =(rot_mat.block(nbsamples,nbsamples,1,2)*sol_y_vec.segment(0, 2) - rot_mat.block(nbsamples,0,1,2)*sol_x_vec.segment(0, 2))(0,0);
  zy+=/*Vpy(0,0)+*/select.VcY(0,0);

  double X;
  double Y;
  zx -= (copdyn.S.block(0,0,1,3) * CoM.x)(0,0);
  X  =  copdyn.UInv(0,0) * zx;
  zy -= (copdyn.S.block(0,0,1,3) * CoM.y)(0,0) ;
  Y  =  copdyn.UInv(0,0) * zy;

  solution.qpSolution.segment(0, nbsamples).fill(X);
  solution.qpSolution.segment(nbsamples, nbsamples).fill(Y);

}

void QPGenerator::buildInequalitiesFeet(const MPCSolution &solution){

  int nbIneq = 5;
  int nbSteps = solution.support_states_vec.back().stepNumber;

  feetInequalities_.resize(nbIneq * nbSteps , nbSteps);

  std::vector<SupportState>::const_iterator prwSS_it = solution.support_states_vec.begin();
  ++prwSS_it;//Point at the first previewed instant
  for( int i = 0; i < mpc_parameters_->nbsamples_qp; ++i ){
    //foot positioning constraints
    if( prwSS_it->state_changed && prwSS_it->stepNumber > 0 && prwSS_it->phase != DS){

      --prwSS_it;//Foot polygons are defined with respect to the supporting foot
      robot_->convexHull(hull, FootHull, *prwSS_it);
      ++prwSS_it;

      int stepNumber = (prwSS_it->stepNumber-1);

      feetInequalities_.DX.block( stepNumber * nbIneq, stepNumber, nbIneq, 1) = hull.A.segment(0, nbIneq);
      feetInequalities_.DY.block( stepNumber * nbIneq, stepNumber, nbIneq, 1) = hull.B.segment(0, nbIneq);
      feetInequalities_.Dc.segment(stepNumber * nbIneq, nbIneq) = hull.D.segment(0, nbIneq);
    }
    ++prwSS_it;
  }

}

void QPGenerator::buildConstraintsFeet(const MPCSolution &solution){

  int nbsteps_previewed = solution.support_states_vec.back().stepNumber;
  const SelectionMatrices &select = preview_->selectionMatrices();
  int nbsamples = mpc_parameters_->nbsamples_qp;

  tmp_mat_.noalias() = feetInequalities_.DX * select.Vf;
  solver_->matrix(matrixA).addTerm(tmp_mat_,  0,  2 * nbsamples);

  tmp_mat_.noalias() = feetInequalities_.DY * select.Vf;
  solver_->matrix(matrixA).addTerm(tmp_mat_,  0, 2 * nbsamples + nbsteps_previewed);

  solver_->vector(vectorBL).addTerm(feetInequalities_.Dc, 0);

  tmp_vec_.noalias() =  feetInequalities_.DX * select.VcfX;
  tmp_vec_ += feetInequalities_.DY * select.VcfY;
  solver_->vector(vectorBL).addTerm(tmp_vec_,  0);

  solver_->vector(vectorBU)().segment(0, tmp_vec_.size()).fill(10e10);
}

void QPGenerator::buildConstraintsCOP(const MPCSolution &solution) {

  int nbsamples = mpc_parameters_->nbsamples_qp;
  std::vector<SupportState>::const_iterator prwSS_it = solution.support_states_vec.begin();

  robot_->convexHull(hull, CoPHull, *prwSS_it, false, false);

  int nbsteps_previewed = solution.support_states_vec.back().stepNumber;
  int size = 2 * nbsamples + 2 * nbsteps_previewed;
  tmp_vec_.resize(size);
  tmp_vec2_.resize(size);

  ++prwSS_it;//Point at the first previewed instant
  for (int i = 0; i < nbsamples; ++i) {
    if (prwSS_it->state_changed) {
      robot_->convexHull(hull, CoPHull, *prwSS_it, false, false);
    }
    tmp_vec_(i)  = std::min(hull.x(0), hull.x(3));
    tmp_vec2_(i) = std::max(hull.x(0), hull.x(3));

    tmp_vec_(nbsamples + i) = std::min(hull.y(0),hull.y(1));
    tmp_vec2_(nbsamples + i)= std::max(hull.y(0),hull.y(1));
    ++prwSS_it;
  }

  tmp_vec_.segment(2 * nbsamples, 2 * nbsteps_previewed).fill(-10e10);
  tmp_vec2_.segment(2 * nbsamples, 2 * nbsteps_previewed).fill(10e10);

  int first_row = 0;
  solver_->vector(vectorXL).addTerm(tmp_vec_, first_row);
  solver_->vector(vectorXU).addTerm(tmp_vec2_, first_row);


}
