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
                         ,ref_(velRef)
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
  int num_recomp = mpc_parameters_->nbFeedbackSamplesStandard();
  Qconst_.resize(num_recomp * nbUsedPonderations);
  QconstN_.resize(num_recomp * nbUsedPonderations);
  choleskyConst_.resize(num_recomp * nbUsedPonderations);
  pconstCoM_.resize(num_recomp * nbUsedPonderations);
  pconstVc_.resize(num_recomp * nbUsedPonderations);
  pconstRef_.resize(num_recomp * nbUsedPonderations);

  int num_samples = mpc_parameters_->nbsamples_qp;
  //CommonMatrixType pondFactor = MatrixXd::Identity(num_samples,num_samples);
  CommonMatrixType G(num_samples, num_samples);

  //TODO: The following has already been done in Walkgen::Init()
  // Define order of variables:
  // --------------------------
  VectorXi order(2 * num_samples);
  for (int i = 0; i < num_samples; ++i) {
    order(i) = 2 * i;
    order(i + num_samples) = 2 * i + 1;
  }
  QPMatrix chol(2 * num_samples, 2 * num_samples);
  chol.rowOrder(order);
  chol.colOrder(order);


  // Precompute:
  // -----------
  for (int i = 0; i < nbUsedPonderations; ++i) {
    for (double period_first = mpc_parameters_->period_mpcsample;
      period_first < mpc_parameters_->period_qpsample+EPSILON;
      period_first += mpc_parameters_->period_mpcsample) {
        int nb = (int)round(period_first / mpc_parameters_->period_mpcsample)-1;
        nb += i * num_recomp;
        robot_->setSelectionNumber(period_first);
        // TODO: Get access to entire vector and do all operations here
        const LinearDynamicsMatrices &CoPDynamics = robot_->body(COM)->dynamics_qp().cop;
        const LinearDynamicsMatrices &VelDynamics = robot_->body(COM)->dynamics_qp().vel;

        //double firstIterationWeight = period_first / mpc_parameters_->period_qpsample;

        tmp_mat_.noalias() = CoPDynamics.UInvT * VelDynamics.UT/**pondFactor*/ * VelDynamics.U * CoPDynamics.UInv;
        tmp_mat2_.noalias() = CoPDynamics.UInvT/**pondFactor*/ * CoPDynamics.UInv;

        G = ponderation_->instantVelocity[i] * tmp_mat_ + ponderation_->JerkMin[i] * tmp_mat2_;
        Qconst_[nb] = G;

        QconstN_[nb] = G + ponderation_->CopCentering[i] * CommonMatrixType::Identity(num_samples, num_samples);

        chol.reset();
        chol.addTerm(QconstN_[nb], 0, 0);
        chol.addTerm(QconstN_[nb], num_samples, num_samples);

        choleskyConst_[nb] = chol.cholesky();

        pconstCoM_[nb] = VelDynamics.S - VelDynamics.U * CoPDynamics.UInv * CoPDynamics.S;
        pconstCoM_[nb] = CoPDynamics.UInvT * VelDynamics.UT * ponderation_->instantVelocity[i]/**pondFactor*/ * pconstCoM_[nb];
        pconstCoM_[nb]-= CoPDynamics.UInvT * ponderation_->JerkMin[i]/**pondFactor*/*CoPDynamics.UInv * CoPDynamics.S;

        pconstVc_[nb]  = CoPDynamics.UInvT * ponderation_->JerkMin[i]/**pondFactor*/ * CoPDynamics.UInv;
        pconstVc_[nb] += CoPDynamics.UInvT * VelDynamics.UT * ponderation_->instantVelocity[i]/**pondFactor*/ * VelDynamics.U * CoPDynamics.UInv;

        pconstRef_[nb] = -CoPDynamics.UInvT * VelDynamics.UT * ponderation_->instantVelocity[i]/**pondFactor*/;
    }
  }

}

void QPGenerator::BuildProblem(MPCSolution &solution) {

  // DIMENSION OF QP:
  // ----------------
  int nbvars = 2 * mpc_parameters_->nbsamples_qp +				// com
    2 * solution.support_states_vec.back().stepNumber;	// Foot placement
  int nbcstr = 5 * solution.support_states_vec.back().stepNumber;	// Foot placement
  solver_->nbvar(nbvars);
  solver_->nbcstr(nbcstr);

  buildObjective(solution);
  buildConstraints(solution);
  if (mpc_parameters_->warmstart) {
    computeWarmStart(solution);//TODO: Weird to modify the solution
  }

}

void QPGenerator::buildObjective(const MPCSolution &solution) {

  // Choose the precomputed element depending on the nb of "feedback-recomputations" until new qp-sample
  int sample_num = mpc_parameters_->nbFeedbackSamplesLeft(solution.support_states_vec[1].previousSamplingPeriod);
  sample_num += ponderation_->activePonderation * mpc_parameters_->nbFeedbackSamplesStandard();

  const BodyState &com = robot_->body(COM)->state();
  const SelectionMatrices &select_mats = preview_->selectionMatrices();
  const CommonMatrixType &rot_mat = preview_->rotationMatrix();
  const CommonMatrixType &rot_mat2 = preview_->rotationMatrix2();

  QPMatrix &Q = solver_->matrix(matrixQ);

  int num_steps_previewed = solution.support_states_vec.back().stepNumber;
  int num_samples = mpc_parameters_->nbsamples_qp;

  
  bool compute_cholesky = (solver_->useCholesky() == true);

  if (!compute_cholesky) {
    Q.addTerm(QconstN_[sample_num], 0, 0);
    Q.addTerm(QconstN_[sample_num], num_samples, num_samples);
  }

  if (num_steps_previewed > 0) {
    tmp_mat_.noalias() = Qconst_[sample_num]*select_mats.V;
    Q.addTerm(tmp_mat_, 0, 2*num_samples);
    Q.addTerm(tmp_mat_, num_samples, 2*num_samples + num_steps_previewed);


    tmp_mat_.noalias() = select_mats.VT * Qconst_[sample_num]*select_mats.V;
    Q.addTerm(tmp_mat_, 2 * num_samples , 2 * num_samples);
    Q.addTerm(tmp_mat_, 2 * num_samples + num_steps_previewed, 2*num_samples + num_steps_previewed);

    if (compute_cholesky == false){
      tmp_mat_.noalias() = select_mats.VT*Qconst_[sample_num];
      Q.addTerm(tmp_mat_, 2*num_samples, 0);
      Q.addTerm(tmp_mat_, 2*num_samples + num_steps_previewed, num_samples);

      // rotate the down left block
      CommonMatrixType dlBlock = Q().block(2*num_samples, 0, 2*num_steps_previewed, 2*num_samples);
      computeMRt(dlBlock, rot_mat2);
      Q().block(2*num_samples, 0, 2*num_steps_previewed, 2*num_samples) = dlBlock;
    }

    // rotate the upper right block
    CommonMatrixType urBlock = Q().block(0, 2*num_samples, 2*num_samples, 2*num_steps_previewed);
    computeRM(urBlock, rot_mat2);
    Q().block(0, 2*num_samples, 2*num_samples, 2*num_steps_previewed) = urBlock;
  }

  if (!compute_cholesky) {
    CommonMatrixType Qmat = Q().block(0, 0, 2 * num_samples, 2 * num_samples);
    Qmat = rot_mat2 * Qmat * rot_mat2.transpose();
    Q().block(0, 0, 2 * num_samples, 2 * num_samples) = Qmat;
  }

  if (compute_cholesky) {
    // rotate the cholesky matrix
    CommonMatrixType chol = choleskyConst_[sample_num];
    rotateCholeskyMatrix(chol, rot_mat2);
    Q.cholesky(chol);
  }

  VectorXd HX(num_samples),HY(num_samples),H(2*num_samples);//TODO: Make this member

  HX = pconstCoM_[sample_num] * com.x;
  HY = pconstCoM_[sample_num] * com.y;

  HX += pconstVc_[sample_num] * select_mats.VcX;
  HY += pconstVc_[sample_num] * select_mats.VcY;

  HX += pconstRef_[sample_num] * ref_->global.x;
  HY += pconstRef_[sample_num] * ref_->global.y;

  if (num_steps_previewed>0){
    tmp_vec_.noalias() = select_mats.VT * HX;
    solver_->vector(vectorP).addTerm(tmp_vec_, 2 * num_samples);
    tmp_vec_.noalias() = select_mats.VT * HY;
    solver_->vector(vectorP).addTerm(tmp_vec_, 2 * num_samples + num_steps_previewed);
  }

  H << HX, HY; //TODO: Unnecessary
  H = rot_mat * H;

  solver_->vector(vectorP).addTerm(H, 0 );

}

void QPGenerator::buildConstraints(const MPCSolution &solution){
  int num_steps_previewed = solution.support_states_vec.back().stepNumber;

  buildConstraintsCOP(solution);
  if (num_steps_previewed>0){
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
  int num_samples = mpc_parameters_->nbsamples_qp;
  solution.initialSolution.resize(4*num_samples + 4*nbSteps);//TODO: 2*num_samples+2*nbSteps
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
  if (size >= 2*num_samples){//TODO: Verification wouldn't be necessary without copying
    // Shift active set by
    solution.initialConstraints.segment(0,         num_samples-1) = initialConstraintTmp.segment(shiftCtr,    num_samples-1);
    solution.initialConstraints.segment(num_samples, num_samples-1) = initialConstraintTmp.segment(shiftCtr+num_samples, num_samples-1);

    // New final ZMP elements are old final elements
    solution.initialConstraints(  num_samples-1) = initialConstraintTmp(  num_samples-1);
    solution.initialConstraints(2*num_samples-1) = initialConstraintTmp(2*num_samples-1);

    // Foot constraints are not shifted
    solution.initialConstraints.segment(2*num_samples          , nbFC*nbSteps)=
      initialConstraintTmp.segment (2*num_samples          , nbFC*nbSteps);
    solution.initialConstraints.segment(2*num_samples+nbFC*nbSteps, nbFC*(nbStepsMax-nbSteps))=
      initialConstraintTmp.segment (2*num_samples+nbFC*nbSteps, nbFC*(nbStepsMax-nbSteps));
  } else {
    solution.initialConstraints = VectorXi::Zero(2*num_samples+(4+nbFC)*nbStepsMax);//TODO: Why this value?
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
  for (int i = 0; i<num_samples; i++){
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
        if (solution.initialConstraints(k+2*num_samples+j*nbFC)!=0){
          int k2=(k+1)%5; // k(4) = k(0)
          if (solution.initialConstraints(k2+2*num_samples+j*nbFC)!=0){
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
      solution.initialSolution(2*num_samples+j) = current_support.x;
      solution.initialSolution(2*num_samples+nbSteps+j) = current_support.y;
      ++j;
    }
    // Place the ZMP on active constraints
    shiftx=shifty=0;
    noActiveConstraints=true;
    int k1=-1;
    int k2=-1;
    if (solution.initialConstraints(0+i*2)==1){
      if (solution.initialConstraints(num_samples+i*2)==1){
        k2=1;
        noActiveConstraints=false;
      }else if (solution.initialConstraints(num_samples+i*2)==2){
        k2=0;
        noActiveConstraints=false;
      }else if (solution.initialConstraints(num_samples+i*2)==0){
        k1=0;
        k2=1;
        noActiveConstraints=false;
      }
    }else if (solution.initialConstraints(0+i*2)==2){
      if (solution.initialConstraints(num_samples+i*2)==1){
        k2=2;
        noActiveConstraints=false;
      }else if (solution.initialConstraints(num_samples+i*2)==2){
        k2=3;
        noActiveConstraints=false;
      }else if (solution.initialConstraints(num_samples+i*2)==0){
        k1=3;
        k2=2;
        noActiveConstraints=false;
      }
    }else if (solution.initialConstraints(num_samples+i*2)==1){
      k1=2;
      k2=1;
      noActiveConstraints=false;
    }else if (solution.initialConstraints(num_samples+i*2)==2){
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
    solution.initialSolution(num_samples+i) = shifty;
    ++prwSS_it;

  }

}

void QPGenerator::computeReferenceVector(const MPCSolution &solution){

  if (ref_->global.x.rows() != mpc_parameters_->nbsamples_qp){
    ref_->global.x.resize(mpc_parameters_->nbsamples_qp);
    ref_->global.y.resize(mpc_parameters_->nbsamples_qp);
  }

  double YawTrunk;
  for (int i=0;i<mpc_parameters_->nbsamples_qp;++i){
    YawTrunk = solution.support_states_vec[i+1].yaw;
    ref_->global.x(i) = ref_->local.x(i)*cos(YawTrunk)-ref_->local.y(i)*sin(YawTrunk);
    ref_->global.y(i) = ref_->local.x(i)*sin(YawTrunk)+ref_->local.y(i)*cos(YawTrunk);
  }

}

void QPGenerator::ConvertCopToJerk(MPCSolution &solution){
  int num_samples = mpc_parameters_->nbsamples_qp;

  const SelectionMatrices &select = preview_->selectionMatrices();
  const CommonMatrixType &rot_mat = preview_->rotationMatrix();
  const BodyState &com = robot_->body(COM)->state();

  VectorXd sol_x_vec = solution.qpSolution.segment(0, num_samples);//TODO: Solution should not be overwritten
  VectorXd sol_y_vec = solution.qpSolution.segment(num_samples, num_samples);
  int nbsteps = solution.support_states_vec.back().stepNumber;
  const VectorXd feet_x_vec = solution.qpSolution.segment(2 * num_samples, nbsteps);
  const VectorXd feet_y_vec = solution.qpSolution.segment(2 * num_samples + nbsteps, nbsteps);


  int nbsamples_interp = 1;
  if (mpc_parameters_->interpolate_whole_horizon == true) {
    nbsamples_interp = num_samples;
  }

  VectorXd &cop_x_vec = solution.cop_prw.pos.x_vec;
  VectorXd &cop_y_vec = solution.cop_prw.pos.y_vec;//TODO: Resize OK?
  cop_x_vec.setZero(num_samples);
  cop_y_vec.setZero(num_samples);
  for (int s = 0; s < nbsamples_interp; ++s) {
    // Rotate: Rx*x - Ry*y;
    cop_x_vec(s) = 
      rot_mat(s, s) * sol_x_vec(s) - rot_mat(s, num_samples + s) * sol_y_vec(s);
    cop_y_vec(s) = 
       rot_mat(num_samples + s, num_samples + s) * sol_y_vec(s) - rot_mat(num_samples + s, s) * sol_x_vec(s);
  }
  // Add to global coordinates of the feet
  cop_x_vec += select.VcX;
  cop_x_vec += select.V * feet_x_vec;
  cop_y_vec += select.VcY; 
  cop_y_vec += select.V * feet_y_vec;

  // Transform to jerk vectors
  const LinearDynamicsMatrices &copdyn = robot_->body(COM)->dynamics_qp().cop;
  VectorXd &jerk_x_vec = solution.com_prw.jerk.x_vec;
  jerk_x_vec.setZero(num_samples); 
  VectorXd &jerk_y_vec = solution.com_prw.jerk.y_vec;
  jerk_y_vec.setZero(num_samples); 
  tmp_vec_ = cop_x_vec - copdyn.S * com.x;
  jerk_x_vec.noalias() = copdyn.UInv * tmp_vec_;
  tmp_vec_ = cop_y_vec - copdyn.S * com.y;
  jerk_y_vec.noalias() = copdyn.UInv * tmp_vec_;

  //TODO: We don't need the rest
  // Vpx(0,0) and Vpy(0,0) always equal to 0 because the first iteration always on the current foot and never on previewed steps
  /*
  const VectorXd px = solution.solution.segment(2*num_samples,         nbSteps);
  const VectorXd py = solution.solution.segment(2*num_samples+nbSteps, nbSteps);
  VectorXd Vpx;
  VectorXd Vpy;
  if (nbSteps>0){
  Vpx =select.V*px;
  Vpy =select.V*py;
  }else{
  Vpx=VectorXd::Zero(num_samples);
  Vpy=VectorXd::Zero(num_samples);
  }
  */

  // Compute ZMP wrt the inertial frame:
  // -----------------------------------
  // TODO: The jerk is computed only for the first instant and copied for the whole preview period!!
  
  double zx;
  zx =(rot_mat.block(0,0,1,2) * sol_x_vec.segment(0, 2) - rot_mat.block(0,num_samples,1,2)*sol_y_vec.segment(0, 2))(0,0);
  zx+=/*Vpx(0,0)+*/select.VcX(0,0);

  double zy;
  zy =(rot_mat.block(num_samples,num_samples,1,2)*sol_y_vec.segment(0, 2) - rot_mat.block(num_samples,0,1,2)*sol_x_vec.segment(0, 2))(0,0);
  zy+=/*Vpy(0,0)+*/select.VcY(0,0);

  double X;
  double Y;
  zx -= (copdyn.S.block(0,0,1,3) * com.x)(0,0);
  X  =  copdyn.UInv(0,0) * zx;
  zy -= (copdyn.S.block(0,0,1,3) * com.y)(0,0) ;
  Y  =  copdyn.UInv(0,0) * zy;

  solution.qpSolution.segment(0, num_samples).fill(X);
  solution.qpSolution.segment(num_samples, num_samples).fill(Y);

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

  int num_steps_previewed = solution.support_states_vec.back().stepNumber;
  const SelectionMatrices &select = preview_->selectionMatrices();
  int num_samples = mpc_parameters_->nbsamples_qp;

  tmp_mat_.noalias() = feetInequalities_.DX * select.Vf;
  solver_->matrix(matrixA).addTerm(tmp_mat_,  0,  2 * num_samples);

  tmp_mat_.noalias() = feetInequalities_.DY * select.Vf;
  solver_->matrix(matrixA).addTerm(tmp_mat_,  0, 2 * num_samples + num_steps_previewed);

  solver_->vector(vectorBL).addTerm(feetInequalities_.Dc, 0);

  tmp_vec_.noalias() =  feetInequalities_.DX * select.VcfX;
  tmp_vec_ += feetInequalities_.DY * select.VcfY;
  solver_->vector(vectorBL).addTerm(tmp_vec_,  0);

  solver_->vector(vectorBU)().segment(0, tmp_vec_.size()).fill(10e10);
}

void QPGenerator::buildConstraintsCOP(const MPCSolution &solution) {

  int num_samples = mpc_parameters_->nbsamples_qp;
  std::vector<SupportState>::const_iterator prwSS_it = solution.support_states_vec.begin();

  robot_->convexHull(hull, CoPHull, *prwSS_it, false, false);

  int num_steps_previewed = solution.support_states_vec.back().stepNumber;
  int size = 2 * num_samples + 2 * num_steps_previewed;
  tmp_vec_.resize(size);
  tmp_vec2_.resize(size);

  ++prwSS_it;//Point at the first previewed instant
  for (int i = 0; i < num_samples; ++i) {
    if (prwSS_it->state_changed) {
      robot_->convexHull(hull, CoPHull, *prwSS_it, false, false);
    }
    tmp_vec_(i)  = std::min(hull.x(0), hull.x(3));
    tmp_vec2_(i) = std::max(hull.x(0), hull.x(3));

    tmp_vec_(num_samples + i) = std::min(hull.y(0),hull.y(1));
    tmp_vec2_(num_samples + i)= std::max(hull.y(0),hull.y(1));
    ++prwSS_it;
  }

  tmp_vec_.segment(2 * num_samples, 2 * num_steps_previewed).fill(-10e10);
  tmp_vec2_.segment(2 * num_samples, 2 * num_steps_previewed).fill(10e10);

  int first_row = 0;
  solver_->vector(vectorXL).addTerm(tmp_vec_, first_row);
  solver_->vector(vectorXU).addTerm(tmp_vec2_, first_row);


}
