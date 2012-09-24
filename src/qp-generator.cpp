#include <mpc-walkgen/qp-generator.h>
#include <mpc-walkgen/qp-matrix.h>
#include <mpc-walkgen/qp-vector.h>
#include <mpc-walkgen/tools.h>

#include <iostream>

using namespace MPCWalkgen;
using namespace Eigen;

QPGenerator::QPGenerator(QPPreview *preview, QPSolver *solver,
		Reference *vel_ref, WeightCoefficients *weight_coefficients,
		RigidBodySystem *robot, const MPCData *mpc_parameters,
		RealClock *clock)
:preview_(preview)
,solver_(solver)
,robot_(robot)
,vel_ref_(vel_ref)
,weight_coefficients_(weight_coefficients)
,mpc_parameters_(mpc_parameters)
,tmp_vec_(1)
,tmp_vec2_(1)
,tmp_mat_(1,1)
,tmp_mat2_(1,1)
,current_time_(0.)
,clock_(clock)
{}

QPGenerator::~QPGenerator() {}


void QPGenerator::PrecomputeObjective() 
{
	//TODO: The following has already been done in Walkgen::Init()
	// Define order of variables:
	// --------------------------
	int num_samples = mpc_parameters_->num_samples_horizon;
	VectorXi order(2 * num_samples);
	for (int i = 0; i < num_samples; ++i) {
		order(i) = 2 * i;
		order(i + num_samples) = 2 * i + 1;
	}
	QPMatrix chol(2 * num_samples, 2 * num_samples);
	chol.rowOrder(order);
	chol.colOrder(order);

	int num_modes = weight_coefficients_->jerk.size();
	int num_recomp = mpc_parameters_->nbFeedbackSamplesStandard();
	Qconst_.resize(num_recomp * num_modes);
	QconstN_.resize(num_recomp * num_modes);
	choleskyConst_.resize(num_recomp * num_modes);
	state_variant_.resize(num_recomp * num_modes);
	select_variant_.resize(num_recomp * num_modes);
	ref_variant_vel_.resize(num_recomp * num_modes);
	ref_variant_pos_.resize(num_recomp * num_modes);

	// Precompute:
	// -----------
	CommonMatrixType G(num_samples, num_samples);
	for (int i = 0; i < num_modes; ++i) {
		for (double first_period = mpc_parameters_->period_mpcsample;
				first_period < mpc_parameters_->period_qpsample + kEps;
				first_period += mpc_parameters_->period_mpcsample) {
			int nb = (int)round(first_period / mpc_parameters_->period_mpcsample)-1;
			nb += i * num_recomp;
			robot_->setSelectionNumber(first_period);//TODO: ?
			// TODO: Get access to entire vector and do all operations here
			const LinearDynamicsMatrices &pos_dyn = robot_->body(COM)->dynamics_qp().pos;
			const LinearDynamicsMatrices &vel_dyn = robot_->body(COM)->dynamics_qp().vel;
			const LinearDynamicsMatrices &cop_dyn = robot_->body(COM)->dynamics_qp().cop;

			tmp_mat_.noalias() = cop_dyn.UInvT * vel_dyn.UT * vel_dyn.U * cop_dyn.UInv;
			G = weight_coefficients_->vel[i] * tmp_mat_;
			tmp_mat_.noalias() = cop_dyn.UInvT * cop_dyn.UInv;
			G += weight_coefficients_->jerk[i] * tmp_mat_;
			tmp_mat_.noalias() = cop_dyn.UInvT * pos_dyn.UT * pos_dyn.U * cop_dyn.UInv;/*position*/
			G +=  weight_coefficients_->pos[i] * tmp_mat_;

			Qconst_[nb] = G;
			QconstN_[nb] = G + weight_coefficients_->cop[i] * CommonMatrixType::Identity(num_samples, num_samples);//TODO: What is difference??

			chol.reset();
			chol.addTerm(QconstN_[nb], 0, 0);
			chol.addTerm(QconstN_[nb], num_samples, num_samples);

			choleskyConst_[nb] = chol.cholesky();

			tmp_mat_.noalias() = vel_dyn.S - vel_dyn.U * cop_dyn.UInv * cop_dyn.S;
			state_variant_[nb] = cop_dyn.UInvT * vel_dyn.UT * weight_coefficients_->vel[i] * tmp_mat_;
			tmp_mat_.noalias() = pos_dyn.S - pos_dyn.U * cop_dyn.UInv * cop_dyn.S;/*position*/
			state_variant_[nb] += cop_dyn.UInvT * pos_dyn.UT * weight_coefficients_->pos[i] * tmp_mat_;/*position*/
			state_variant_[nb] -= cop_dyn.UInvT * weight_coefficients_->jerk[i] * cop_dyn.UInv * cop_dyn.S;

			select_variant_[nb]  = cop_dyn.UInvT * weight_coefficients_->jerk[i] * cop_dyn.UInv;
			select_variant_[nb] += cop_dyn.UInvT * vel_dyn.UT * weight_coefficients_->vel[i] * vel_dyn.U * cop_dyn.UInv;
			select_variant_[nb] += cop_dyn.UInvT * pos_dyn.UT * weight_coefficients_->pos[i] * pos_dyn.U * cop_dyn.UInv;/*position*/

			ref_variant_vel_[nb] = -cop_dyn.UInvT * vel_dyn.UT * weight_coefficients_->vel[i];
			ref_variant_pos_[nb] = -cop_dyn.UInvT * pos_dyn.UT * weight_coefficients_->pos[i];/*position*/
		}
	}

}

void QPGenerator::BuildProblem(MPCSolution &solution) 
{
	// DIMENSION OF QP:
	// ----------------
	int nbvars = 2 * mpc_parameters_->num_samples_horizon +				// com
			2 * solution.support_states_vec.back().stepNumber;	// Foot placement
	int num_constr = 5 * solution.support_states_vec.back().stepNumber;	// Foot placement
	solver_->nbvar(nbvars);
	solver_->num_constr(num_constr);

	BuildObjective(solution);
	buildConstraints(solution);
	if (mpc_parameters_->warmstart) {
		computeWarmStart(solution);//TODO: Weird to modify the solution
	}

}

void QPGenerator::BuildObjective(const MPCSolution &solution) {


	// Choose the precomputed element depending on the nb of "feedback-recomputations" until new qp-sample
	int sample_num = mpc_parameters_->nbFeedbackSamplesLeft(solution.support_states_vec[1].previousSamplingPeriod);
	sample_num += weight_coefficients_->active_mode * mpc_parameters_->nbFeedbackSamplesStandard();

	const BodyState &com = robot_->body(COM)->state();
	const SelectionMatrices &select_mats = preview_->selectionMatrices();
	const CommonMatrixType &rot_mat = preview_->rotationMatrix();
	const CommonMatrixType &rot_mat2 = preview_->rotationMatrix2();
	const CommonMatrixType &rot_mat2_trans = preview_->rotationMatrix2Trans();

	QPMatrix &Q = solver_->matrix(matrixQ);

	int num_steps_previewed = solution.support_states_vec.back().stepNumber;
	int num_samples = mpc_parameters_->num_samples_horizon;

	if (!solver_->useCholesky()) {//TODO: This part is the most costly >50%
		Q.addTerm(QconstN_[sample_num], 0, 0);
		Q.addTerm(QconstN_[sample_num], num_samples, num_samples);

		CommonMatrixType Qmat = Q().block(0, 0, 2 * num_samples, 2 * num_samples);
		tmp_mat_.noalias() = rot_mat2 * Qmat * rot_mat2_trans;
		Q().block(0, 0, 2 * num_samples, 2 * num_samples) = tmp_mat_;
	} else {
		// rotate the cholesky matrix
		CommonMatrixType chol = choleskyConst_[sample_num];
		rotateCholeskyMatrix(chol, rot_mat2);
		Q.cholesky(chol);
	}

	if (num_steps_previewed > 0) {
		tmp_mat_.noalias() = Qconst_[sample_num] * select_mats.sample_step;
		Q.addTerm(tmp_mat_, 0, 2 * num_samples);
		Q.addTerm(tmp_mat_, num_samples, 2 * num_samples + num_steps_previewed);

		tmp_mat_.noalias() = select_mats.sample_step_trans * Qconst_[sample_num] * select_mats.sample_step;
		Q.addTerm(tmp_mat_, 2 * num_samples, 2 * num_samples);
		Q.addTerm(tmp_mat_, 2 * num_samples + num_steps_previewed, 2 * num_samples + num_steps_previewed);


		if (!solver_->useCholesky()) {
			tmp_mat_.noalias() = select_mats.sample_step_trans * Qconst_[sample_num];
			Q.addTerm(tmp_mat_, 2 * num_samples, 0);
			Q.addTerm(tmp_mat_, 2 * num_samples + num_steps_previewed, num_samples);

			int timer_rotate = clock_->StartCounter();
			// rotate the lower left block
			// TODO(andrei): Rotation can be done before addition
			CommonMatrixType dlBlock = Q().block(2 * num_samples, 0, 2 * num_steps_previewed, 2 * num_samples);
			computeMRt(dlBlock, rot_mat2);
			Q().block(2 * num_samples, 0, 2 * num_steps_previewed, 2 * num_samples) = dlBlock;
			clock_->StopCounter(timer_rotate);
		}

		int timer_rotate = clock_->StartCounter();
		// rotate the upper right block
		// TODO(efficiency): Rotation can be done before addition
		CommonMatrixType urBlock = Q().block(0, 2 * num_samples, 2*num_samples, 2*num_steps_previewed);
		computeRM(urBlock, rot_mat2);
		Q().block(0, 2 * num_samples, 2 * num_samples, 2 * num_steps_previewed) = urBlock;
		clock_->StopCounter(timer_rotate);
	}

	int timer_buildobjective = clock_->StartCounter();
	VectorXd HX(num_samples), HY(num_samples), H(2 * num_samples);//TODO: Make this member

	CommonVectorType state_x(mpc_parameters_->dynamics_order), state_y(mpc_parameters_->dynamics_order);
	for (int i = 0; i < mpc_parameters_->dynamics_order; i++) {
		state_x(i) = com.x(i);
		state_y(i) = com.y(i);
	}

	HX = state_variant_[sample_num] * state_x;
	HY = state_variant_[sample_num] * state_y;


	HX += select_variant_[sample_num] * select_mats.sample_step_cx;
	HY += select_variant_[sample_num] * select_mats.sample_step_cy;

	HX += ref_variant_vel_[sample_num] * vel_ref_->global.x;
	HY += ref_variant_vel_[sample_num] * vel_ref_->global.y;

	//HX += ref_variant_pos_[sample_num] * vel_ref_->global.x;
	//HY += ref_variant_pos_[sample_num] * vel_ref_->global.y;

	if (num_steps_previewed > 0) {
		tmp_vec_.noalias() = select_mats.sample_step_trans * HX;
		solver_->vector(vectorP).addTerm(tmp_vec_, 2 * num_samples);
		tmp_vec_.noalias() = select_mats.sample_step_trans * HY;
		solver_->vector(vectorP).addTerm(tmp_vec_, 2 * num_samples + num_steps_previewed);
	}

	H << HX, HY; //TODO: Unnecessary if rot_mat half the size
	H = rot_mat * H;

	solver_->vector(vectorP).addTerm(H, 0 );
	clock_->StopCounter(timer_buildobjective);
}

void QPGenerator::buildConstraints(const MPCSolution &solution){
	int num_steps_previewed = solution.support_states_vec.back().stepNumber;

	buildConstraintsCOP(solution);
	if (num_steps_previewed>0){
		BuildInequalitiesFeet(solution);
		BuildConstraintsFeet(solution);
		//BuildFootVelConstraints(solution);
	}
}

void QPGenerator::computeWarmStart(MPCSolution &solution){

	// Initialize:
	// -----------
	int num_steps = solution.support_states_vec.back().stepNumber;
	int nbStepsMax = mpc_parameters_->num_samples_horizon;

	int nbFC = 5;// Number of foot constraints per step TODO: can be read?
	int num_samples = mpc_parameters_->num_samples_horizon;
	solution.initialSolution.resize(4*num_samples + 4*num_steps);//TODO: 2*num_samples+2*num_steps
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
		solution.initialConstraints(2 * num_samples-1) = initialConstraintTmp(2*num_samples-1);

		// Foot constraints are not shifted
		solution.initialConstraints.segment(2 * num_samples, nbFC*num_steps)=
				initialConstraintTmp.segment (2 * num_samples, nbFC*num_steps);
		solution.initialConstraints.segment(2 * num_samples + nbFC*num_steps, nbFC * (nbStepsMax - num_steps))=
				initialConstraintTmp.segment (2 * num_samples + nbFC * num_steps, nbFC * (nbStepsMax - num_steps));
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
		robot_->convexHull(cop_hull_edges_, CoPHull, *prwSS_it, false);

		// Check if the support foot has changed
		if (prwSS_it->state_changed && prwSS_it->stepNumber>0){

			// Get feet convex hull for current support
			prwSS_it--;
			robot_->convexHull(foot_hull_edges_, FootHull,*prwSS_it, false);
			prwSS_it++;

			// Place the foot on active constraints
			shiftx = shifty = 0;
			noActiveConstraints = true;
			for(int k = 0; k < nbFC; ++k){
				if (solution.initialConstraints(k+2*num_samples+j*nbFC)!=0){
					int k2 = (k+1)%5; // k(4) = k(0)
					if (solution.initialConstraints(k2+2*num_samples+j*nbFC)!=0){
						shiftx=foot_hull_edges_.x(k2);
						shifty=foot_hull_edges_.y(k2);
					}else{
						shiftx = (foot_hull_edges_.x(k) + foot_hull_edges_.x(k2)) / 2.;
						shifty = (foot_hull_edges_.y(k) + foot_hull_edges_.y(k2)) / 2.;
					}
					noActiveConstraints = false;
					break;
				}
			}
			if (noActiveConstraints){
				shiftx = (foot_hull_edges_.x(4) + foot_hull_edges_.x(0)) / 2.;
				shifty = (foot_hull_edges_.y(4) + foot_hull_edges_.y(2)) / 2.;
			}

			current_support.x += shiftx;
			current_support.y += shifty;

			// Set the new position into initial solution vector
			solution.initialSolution(2*num_samples+j) = current_support.x;
			solution.initialSolution(2*num_samples+num_steps+j) = current_support.y;
			++j;
		}
		// Place the ZMP on active constraints
		shiftx=shifty=0;
		noActiveConstraints = true;
		int k1=-1;
		int k2=-1;
		if (solution.initialConstraints(0+i*2)==1){
			if (solution.initialConstraints(num_samples+i*2)==1){
				k2=1;
				noActiveConstraints = false;
			} else if (solution.initialConstraints(num_samples+i*2)==2){
				k2=0;
				noActiveConstraints = false;
			} else if (solution.initialConstraints(num_samples+i*2)==0){
				k1=0;
				k2=1;
				noActiveConstraints = false;
			}
		}else if (solution.initialConstraints(0+i*2)==2){
			if (solution.initialConstraints(num_samples+i*2)==1){
				k2=2;
				noActiveConstraints = false;
			}else if (solution.initialConstraints(num_samples+i*2)==2){
				k2=3;
				noActiveConstraints = false;
			}else if (solution.initialConstraints(num_samples+i*2)==0){
				k1=3;
				k2=2;
				noActiveConstraints = false;
			}
		}else if (solution.initialConstraints(num_samples+i*2)==1){
			k1=2;
			k2=1;
			noActiveConstraints = false;
		}else if (solution.initialConstraints(num_samples+i*2)==2){
			k1=0;
			k2=3;
			noActiveConstraints = false;
		}

		if (!noActiveConstraints){
			if (k1!=-1){
				shiftx=(cop_hull_edges_.x[k1]+cop_hull_edges_.x[k2])/2;
				shifty=(cop_hull_edges_.y[k1]+cop_hull_edges_.y[k2])/2;
			}else{
				shiftx=cop_hull_edges_.x[k2];
				shifty=cop_hull_edges_.y[k2];
			}
		}

		solution.initialSolution(i) = shiftx;
		solution.initialSolution(num_samples+i) = shifty;
		++prwSS_it;

	}

}

void QPGenerator::computeReferenceVector(const MPCSolution &solution)
{
	if (vel_ref_->global.x.rows() != mpc_parameters_->num_samples_horizon){
		vel_ref_->global.x.resize(mpc_parameters_->num_samples_horizon);
		vel_ref_->global.y.resize(mpc_parameters_->num_samples_horizon);
	}

	double YawTrunk;
	for (int i = 0; i < mpc_parameters_->num_samples_horizon; ++i){
		YawTrunk = solution.support_states_vec[i+1].yaw;
		vel_ref_->global.x(i) = vel_ref_->local.x(i) * cos(YawTrunk) - vel_ref_->local.y(i) * sin(YawTrunk);
		vel_ref_->global.y(i) = vel_ref_->local.x(i) * sin(YawTrunk) + vel_ref_->local.y(i) * cos(YawTrunk);
	}
}

//TODO(performance): Optimize this
void QPGenerator::ConvertCopToJerk(MPCSolution &solution)
{
	int num_samples = mpc_parameters_->num_samples_horizon;

	const SelectionMatrices &select = preview_->selectionMatrices();
	const CommonMatrixType &rot_mat = preview_->rotationMatrix();
	const BodyState &com = robot_->body(COM)->state();

	VectorXd sol_x_vec = solution.qp_solution_vec.segment(0, num_samples);//TODO: Solution should not be overwritten
	VectorXd sol_y_vec = solution.qp_solution_vec.segment(num_samples, num_samples);//TODO(performance): sol_x/y_vec has to be
	int num_steps = solution.support_states_vec.back().stepNumber;
	const VectorXd feet_x_vec = solution.qp_solution_vec.segment(2 * num_samples, num_steps);
	const VectorXd feet_y_vec = solution.qp_solution_vec.segment(2 * num_samples + num_steps, num_steps);


	int num_samples_interp = 1;
	if (mpc_parameters_->interpolate_whole_horizon == true) {
		num_samples_interp = num_samples;
	}

	VectorXd &cop_x_vec = solution.cop_prw.pos.x_vec;
	VectorXd &cop_y_vec = solution.cop_prw.pos.y_vec;//TODO: Resize OK?
	//cop_x_vec.setZero(num_samples);//TODO(performance): Resize necessary?
	//cop_y_vec.setZero(num_samples);
	for (int s = 0; s < num_samples_interp; ++s) {
		// Rotate: Rx*x - Ry*y;
		cop_x_vec(s) =  rot_mat(s, s) * sol_x_vec(s) - rot_mat(s, num_samples + s) * sol_y_vec(s);
		cop_x_vec(s) += select.sample_step_cx(s);
		cop_y_vec(s) =  rot_mat(num_samples + s, num_samples + s) * sol_y_vec(s) - rot_mat(num_samples + s, s) * sol_x_vec(s);
		cop_y_vec(s) += select.sample_step_cy(s);
	}
	// Add to global coordinates of the feet
	cop_x_vec += select.sample_step * feet_x_vec;//TODO(performance): Optimize this for first interpolate_whole_horizon == false
	cop_y_vec += select.sample_step * feet_y_vec;

	CommonVectorType state_x(mpc_parameters_->dynamics_order), state_y(mpc_parameters_->dynamics_order);
	for (int i = 0; i < mpc_parameters_->dynamics_order; i++) {
		state_x(i) = com.x(i);
		state_y(i) = com.y(i);
	}

	//TODO(performance): Optimize this for first interpolate_whole_horizon == false
	// Transform to com motion
	const LinearDynamicsMatrices &copdyn = robot_->body(COM)->dynamics_qp().cop;
	VectorXd &jerk_x_vec = solution.com_prw.jerk.x_vec;
	//jerk_x_vec.setZero(num_samples);
	VectorXd &jerk_y_vec = solution.com_prw.jerk.y_vec;
	//jerk_y_vec.setZero(num_samples);
	tmp_vec_ = cop_x_vec - copdyn.S * state_x;
	jerk_x_vec.noalias() = copdyn.UInv * tmp_vec_;
	tmp_vec_ = cop_y_vec - copdyn.S * state_y;
	jerk_y_vec.noalias() = copdyn.UInv * tmp_vec_;

}

void QPGenerator::BuildInequalitiesFeet(const MPCSolution &solution) {

	int num_ineqs = 5;
	int num_steps = solution.support_states_vec.back().stepNumber;

	feetInequalities_.resize(num_ineqs * num_steps , num_steps);

	std::vector<SupportState>::const_iterator prwSS_it = solution.support_states_vec.begin();
	++prwSS_it;//Point at the first previewed instant
	for( int i = 0; i < mpc_parameters_->num_samples_horizon; ++i ){
		//foot positioning constraints
		if( prwSS_it->state_changed && prwSS_it->stepNumber > 0 && prwSS_it->phase != DS){

			--prwSS_it;//Foot polygons are defined with respect to the supporting foot
			robot_->convexHull(hull, FootHull, *prwSS_it);
			++prwSS_it;

			int stepNumber = (prwSS_it->stepNumber-1);

			feetInequalities_.DX.block( stepNumber * num_ineqs, stepNumber, num_ineqs, 1) = hull.A.segment(0, num_ineqs);
			feetInequalities_.DY.block( stepNumber * num_ineqs, stepNumber, num_ineqs, 1) = hull.B.segment(0, num_ineqs);
			feetInequalities_.Dc.segment(stepNumber * num_ineqs, num_ineqs) = hull.D.segment(0, num_ineqs);
		}
		++prwSS_it;
	}

}

void QPGenerator::BuildConstraintsFeet(const MPCSolution &solution){

	int num_steps_previewed = solution.support_states_vec.back().stepNumber;
	const SelectionMatrices &select = preview_->selectionMatrices();
	int num_samples = mpc_parameters_->num_samples_horizon;

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

void QPGenerator::BuildFootVelConstraints(const MPCSolution &solution)
{
	assert(robot_->robot_data().max_foot_vel > kEps);

	double raise_time = 0.05;//TODO: Hard coded value has to come from robot_
	double time_left = solution.support_states_vec.front().time_limit - raise_time - current_time_;
	double max_vel = robot_->robot_data().max_foot_vel;

	int num_steps_previewed = solution.support_states_vec.back().stepNumber;
	int x_var_pos = mpc_parameters_->num_samples_horizon;
	int y_var_pos = mpc_parameters_->num_samples_horizon; + num_steps_previewed;

	const BodyState *flying_foot;
	const SupportState &current_support = solution.support_states_vec.front();
	if (current_support.foot == LEFT) {
		flying_foot = &robot_->body(RIGHT_FOOT)->state();
	} else {
		flying_foot = &robot_->body(LEFT_FOOT)->state();
	}

	double upper_limit_x = max_vel * time_left + flying_foot->x(0);
	double upper_limit_y = max_vel * time_left + flying_foot->y(0);
	solver_->vector(vectorXU).addTerm(upper_limit_x, x_var_pos);
	solver_->vector(vectorXU).addTerm(upper_limit_y, y_var_pos);
	double lower_limit_x = -max_vel * time_left + flying_foot->x(0);
	double lower_limit_y = -max_vel * time_left + flying_foot->y(0);
	solver_->vector(vectorXL).addTerm(lower_limit_x, x_var_pos);
	solver_->vector(vectorXL).addTerm(lower_limit_y, y_var_pos);
	//std::cout << "x: " << lower_limit_x <<  " : " << upper_limit_x <<std::endl;
	std::cout << "y:"<<flying_foot->y(0)<<" " << lower_limit_y <<  " : " << upper_limit_y <<std::endl;
}

void QPGenerator::buildConstraintsCOP(const MPCSolution &solution) 
{
	int num_samples = mpc_parameters_->num_samples_horizon;
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
