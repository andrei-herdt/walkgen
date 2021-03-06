#include <mpc-walkgen/walkgen.h>

#include <iostream>

using namespace MPCWalkgen;
using namespace std;

Walkgen::Walkgen()
: mpc_parameters_()
,robot_data_()
,solver_(NULL)
,builder_(NULL)
,heur_preview_(NULL)
,robot_()
,orient_preview_(NULL)
,clock_()
,output_()
,output_index_(0)
,solution_()
,vel_ref_()
,first_coarse_sample_(0.)
,first_fine_sample_(0.)
,next_computation_(0)
,next_act_sample_(0)
,current_time_(0)
,last_des_cop_x_(0)
,last_des_cop_y_(0)
,last_first_contr_x_(0)
,last_first_contr_y_(0){
	orient_preview_ = new OrientationsPreview();
}


Walkgen::~Walkgen() {
	if (orient_preview_ != NULL) {
		delete orient_preview_;
		orient_preview_ = NULL;
	}

	if (solver_ != NULL) {
		delete solver_;
		solver_ = NULL;
	}

	if (builder_ != NULL) {
		delete builder_;
		builder_ = NULL;
	}

	if (heur_preview_ != NULL) {
		delete heur_preview_;
		heur_preview_ = NULL;
	}

}

void Walkgen::Init(MPCParameters &mpc_parameters) {
	assert(mpc_parameters_.period_dsss > kEps);
	assert(mpc_parameters.num_samples_horizon_max > mpc_parameters_.num_steps_ssds);
	assert(mpc_parameters.period_qpsample >= mpc_parameters.period_recomputation - kEps);
	assert(mpc_parameters.period_recomputation >= mpc_parameters.period_actsample - kEps);

	mpc_parameters_ = mpc_parameters;

	robot_.Init(&mpc_parameters_);

	// Solver:
	// -------
	int num_constr_step = 5; // TODO: Move this to MPCParameters
	int num_steps_max = mpc_parameters_.num_steps_max;
	int num_vars_max = 2 * (mpc_parameters_.num_samples_horizon_max + num_steps_max);
	int num_constr_max = 0;
	if (mpc_parameters_.formulation == DECOUPLED_MODES) {
		num_vars_max += 2;
		num_constr_max += 2;
	}
	num_constr_max += num_constr_step * num_steps_max;
	// Constraints on foot placement
	num_constr_max += 2;

	solver_ = createQPSolver(mpc_parameters_.solver, num_vars_max,  num_constr_max);

	/*
	// Set order of optimization variables
	Eigen::VectorXi order(solver_->num_var_max());
	for (int i = 0; i < mpc_parameters_.num_samples_horizon; ++i) {// 0,2,4,1,3,5 (CoM)
		order(i) = 2 * i;
		order(i + mpc_parameters_.num_samples_horizon) = 2*i+1;
	}
	for (int i = 2 * mpc_parameters_.num_samples_horizon; i < solver_->num_var_max(); ++i) {// 6,7,8 (Feet)
		order(i) = i;
	}
	solver_->SetVarIndices(order);
	 */

	// Resize:
	// -------
	int num_unstable_modes = 0;
	if (mpc_parameters_.formulation == DECOUPLED_MODES) {
		num_unstable_modes = 1;
	}
	int num_samples_st = max(mpc_parameters_.num_samples_state, mpc_parameters_.num_samples_horizon_max);
	solution_.com_act.SetZero(mpc_parameters_.num_samples_act(), num_unstable_modes);
	solution_.com_prw.SetZero(num_samples_st, num_unstable_modes);

	solution_.pos_ref.SetZero(num_samples_st);		//DEPRECATED:

	pos_ref_.SetZero(num_samples_st);
	vel_ref_.SetZero(num_samples_st);
	cp_ref_.SetZero(num_samples_st);
	new_vel_ref_.SetZero(num_samples_st);

	heur_preview_ = new HeuristicPreview(&vel_ref_, &robot_, &mpc_parameters_, &clock_);

	builder_= new QPBuilder(heur_preview_, solver_, &pos_ref_, &vel_ref_, &cp_ref_,
			&robot_, &mpc_parameters_, &clock_, &last_des_cop_x_, &last_des_cop_y_,
			&last_first_contr_x_, &last_first_contr_y_);

	if (mpc_parameters_.init_com_height > kEps) {
		robot_.com()->state().z[POSITION] = mpc_parameters_.init_com_height;
		robot_.ComputeDynamics();
		builder_->PrecomputeObjective();
	}

}

void Walkgen::Init(const RobotData &robot_data) {
	robot_data_ = robot_data;
	robot_.Init(robot_data_);

	last_des_cop_x_ = robot_data_.com[X];
	last_des_cop_y_ = robot_data_.com[Y];
	last_first_contr_x_ = robot_data_.com[X] - robot_data_.left_foot.position[X];//TODO: Left foot first assumption
	last_first_contr_y_ = robot_data_.com[Y] - robot_data_.left_foot.position[Y];
	Init();
}

const MPCSolution &Walkgen::Go(){
	current_time_ += mpc_parameters_.period_recomputation;
	return Go(current_time_);
}

const MPCSolution &Walkgen::Go(double time){
	current_time_ = time;

	if (time > next_computation_ - mpc_parameters_.period_recomputation/2.) {
		next_computation_ += mpc_parameters_.period_recomputation;
		if (time > next_computation_ - mpc_parameters_.period_recomputation/2.) {
			ResetCounters(time);
		}
		if(time > first_fine_sample_ - mpc_parameters_.period_recomputation/2. && mpc_parameters_.period_inter_samples > kEps){
			first_fine_sample_ += mpc_parameters_.period_inter_samples;
			if (time > first_fine_sample_ - mpc_parameters_.period_recomputation/2.) {
				ResetCounters(time);
			}
		}
		if(time > first_coarse_sample_ - mpc_parameters_.period_recomputation/2.){
			first_coarse_sample_ += mpc_parameters_.period_qpsample;
			if (time > first_coarse_sample_ - mpc_parameters_.period_recomputation/2.) {
				ResetCounters(time);
			}
		}
		ResetOutputIndex();
		builder_->current_time(current_time_);

		//int problem_counter = clock_.StartCounter();
		BuildProblem();
		//clock_.StopCounter(problem_counter);

		if (mpc_parameters_.problem_dumping) {
			solver_->DumpProblem("problem", current_time_, "txt");
		}
		//int solver_counter = clock_.StartCounter();
		solver_->Solve(solution_, mpc_parameters_.warmstart, mpc_parameters_.solver.analysis);
		//clock_.StopCounter(solver_counter);

		//int traj_counter = clock_.StartCounter();
		GenerateTrajectories();
		//clock_.StopCounter(traj_counter);

		// Store parts of the solution:
		// ----------------------------
		StoreResult();
	}

	if (time > next_act_sample_ - kEps) {
		next_act_sample_ += mpc_parameters_.period_actsample;

		IncrementOutputIndex();
		UpdateOutput();
	}

	return solution_;
}

//
// Private methods:
//
void Walkgen::Init() {
	if (mpc_parameters_.init_com_height < kEps) {
		robot_.ComputeDynamics();
		builder_->PrecomputeObjective();
	}
	orient_preview_->Init(&mpc_parameters_, robot_data_);
	mpc_parameters_.penalties.active_mode = 0;
	if (mpc_parameters_.solver.name == QPOASES) {
		BuildProblem();

		solver_->Init();
	}
}

void Walkgen::SetCounters(double time) {}

void Walkgen::ResetCounters(double time) {
	first_coarse_sample_ = time + mpc_parameters_.period_qpsample;
	first_fine_sample_ = time + mpc_parameters_.period_inter_samples;
	next_computation_ = time + mpc_parameters_.period_recomputation;
	next_act_sample_ = time + mpc_parameters_.period_actsample;
}

void Walkgen::BuildProblem() {
	// UPDATE INTERNAL DATA:
	// ---------------------
	solver_->Reset();
	solution_.Reset();
	vel_ref_ = new_vel_ref_;

	solution_.first_coarse_period = first_coarse_sample_ - current_time_;//TODO:
	double first_fine_period = first_fine_sample_ - current_time_;//TODO:

	// PREVIEW:
	// --------
	heur_preview_->PreviewSamplingTimes(current_time_, first_fine_period, solution_.first_coarse_period, solution_);

	if (mpc_parameters_.mapping == ZERO_MAP) {
		mpc_parameters_.num_samples_contr = static_cast<int>(solution_.sampling_times_vec.size() - 1);
		mpc_parameters_.num_samples_state = mpc_parameters_.num_samples_contr;
	}
	heur_preview_->PreviewSupportStates(first_fine_period, solution_);

	SetWalkingMode();

	// Modify global cp reference in the frontal plane:
	// ------------------------------------------------
	// TODO: Left foot first supposition for the cp references!!
	// TODO: Own method needed.
	double supp_time_passed = 0.;
	double omega = sqrt(kGravity / robot_.com()->state().z[0]); 
	double cop_center_dis_ss = 0.0;
	//if (mpc_parameters_.penalties.dcop_online) {
	//	cop_center_dis_ss = 0.;//mpc_parameters_.cop_off;
	//}
	cp_ref_.offset_ss[Y] = (robot_data_.left_foot.position[Y] - robot_data_.right_foot.position[Y] - 2. * cop_center_dis_ss)
							/ (exp(omega * (mpc_parameters_.period_ss + mpc_parameters_.period_trans_ds())) + 1.);
	if (mpc_parameters_.walking_mode.type == INITIAL) {
		cp_ref_.init[X] = robot_data_.com[X];
		cp_ref_.init[Y] = robot_data_.com[Y];
	} else if (mpc_parameters_.walking_mode.type == WALK_START) {
		cp_ref_.init[X] = robot_data_.com[X];
		cp_ref_.init[Y] = robot_.com()->state().y[POSITION] + 1. / omega * robot_.com()->state().y[VELOCITY];
		cp_ref_.offset_ds[Y] = (robot_data_.left_foot.position[Y] - cop_center_dis_ss - cp_ref_.offset_ss[Y] - cp_ref_.init[Y]) / exp(omega * (mpc_parameters_.period_dsss));
	}

	if (mpc_parameters_.walking_mode.type == INITIAL) {
		cp_ref_.global.x.fill(cp_ref_.init[X]);
		cp_ref_.local.x.fill(cp_ref_.init[X] - robot_data_.left_foot.position[X]);
		cp_ref_.z_ref_x = cp_ref_.init[X];
		cp_ref_.global.y.fill(cp_ref_.init[Y]);
		cp_ref_.local.y.fill(cp_ref_.init[Y] - robot_data_.left_foot.position[Y]);
		cp_ref_.z_ref_y = cp_ref_.init[Y];
	} else if (mpc_parameters_.walking_mode.type == WALK || mpc_parameters_.walking_mode.type == WALK_START) {
		cp_ref_.global.x.fill(cp_ref_.init[X]);
		cp_ref_.local.x.fill(cp_ref_.init[X] - robot_data_.left_foot.position[X]);
		cp_ref_.z_ref_x = cp_ref_.init[X];
		for (int i = 0; i < mpc_parameters_.num_samples_state ; i++) {
			if (solution_.support_states_vec[i + 1].phase == SS) {
				supp_time_passed = solution_.sampling_times_vec[i + 1] - solution_.support_states_vec[i + 1].start_time;
				if (solution_.support_states_vec[i + 1].foot == LEFT) {
					cp_ref_.global.y[i] = robot_data_.left_foot.position[Y] - cop_center_dis_ss - exp(omega * supp_time_passed) * cp_ref_.offset_ss[Y];
					cp_ref_.local.y[i] = - cop_center_dis_ss - exp(omega * supp_time_passed) * cp_ref_.offset_ss[Y];
					cp_ref_.z_ref_y = robot_data_.left_foot.position[Y] - cop_center_dis_ss;
				} else if (solution_.support_states_vec[i + 1].foot == RIGHT) {
					cp_ref_.global.y[i] = robot_data_.right_foot.position[Y] + cop_center_dis_ss + exp(omega * supp_time_passed) * cp_ref_.offset_ss[Y];
					cp_ref_.local.y[i] = cop_center_dis_ss + exp(omega * supp_time_passed) * cp_ref_.offset_ss[Y];
					cp_ref_.z_ref_y = robot_data_.right_foot.position[Y] + cop_center_dis_ss;
				}
			} else {
				supp_time_passed = solution_.sampling_times_vec[i + 1] - mpc_parameters_.walking_mode.start_time;
				cp_ref_.global.y[i] = cp_ref_.init[Y] + exp(omega * supp_time_passed) * cp_ref_.offset_ds[Y];
				cp_ref_.z_ref_y = cp_ref_.init[Y];
				cp_ref_.local.y[i] = -(robot_data_.left_foot.position[1] - cp_ref_.init[Y]) + exp(omega * supp_time_passed) * cp_ref_.offset_ds[Y];
			}
		}
	}

	// Last reference is foot center
	//cp_ref_.local.x[mpc_parameters_.num_samples_horizon - 1] = 0.;
	//cp_ref_.local.y[mpc_parameters_.num_samples_horizon - 1] = 0.;
	//if (solution_.support_states_vec[mpc_parameters_.num_samples_horizon].phase == DS) {
	//	if (solution_.support_states_vec[mpc_parameters_.num_samples_horizon].foot == LEFT) {
	//		cp_ref_.local.y[mpc_parameters_.num_samples_horizon - 1] = - robot_data_.lateral_ds_feet_dist;
	//	} else {
	//		cp_ref_.local.y[mpc_parameters_.num_samples_horizon - 1] = robot_data_.lateral_ds_feet_dist;
	//	}
	//}

	orient_preview_->preview_orientations( current_time_, vel_ref_,
			mpc_parameters_.period_ss, robot_.left_foot()->state(),
			robot_.right_foot()->state(), solution_ );
	heur_preview_->BuildRotationMatrix(solution_);

	// BUILD:
	// ------
	builder_->BuildGlobalVelocityReference(solution_);

	builder_->BuildProblem(solution_);

}

void Walkgen::GenerateTrajectories() {
	builder_->TransformControlVector(solution_);	//Half of the time spent here. Can be reduced.

	robot_.Interpolate(solution_, current_time_, vel_ref_);

	robot_.UpdateState(solution_);

	orient_preview_->InterpolateTrunkYaw(&robot_);//TODO: Change this

	next_act_sample_ += mpc_parameters_.period_actsample;
	ResetOutputIndex();
	UpdateOutput();
}

void Walkgen::ResetOutputIndex() {
	output_index_ = 0;
}

void Walkgen::IncrementOutputIndex() {
	assert(output_index_ < mpc_parameters_.num_samples_act() - 1);

	output_index_++;
}

void Walkgen::UpdateOutput() {
	output_.com.x 	= solution_.com_act.pos.x_vec[output_index_];
	output_.com.y 	= solution_.com_act.pos.y_vec[output_index_];
	output_.com.z 	= robot_.com()->state().z(0);
	output_.com.dx 	= solution_.com_act.vel.x_vec[output_index_];
	output_.com.dy 	= solution_.com_act.vel.y_vec[output_index_];
	output_.com.dz 	= robot_.com()->state().z(1);
	output_.com.ddx = solution_.com_act.acc.x_vec[output_index_];
	output_.com.ddy = solution_.com_act.acc.y_vec[output_index_];
	output_.com.ddz = robot_.com()->state().z(2);

	output_.cop.x = solution_.com_act.cop.x_vec[output_index_];
	output_.cop.y = solution_.com_act.cop.y_vec[output_index_];

	last_des_cop_x_ = output_.cop.x;
	last_des_cop_y_ = output_.cop.y;
	last_first_contr_x_ = solution_.qp_solution_vec[0];
	int num_unst_modes = 0;
	if (mpc_parameters_.formulation == DECOUPLED_MODES) {
		num_unst_modes = 1;
	}
	last_first_contr_y_ = solution_.qp_solution_vec[mpc_parameters_.num_samples_contr + num_unst_modes];

	output_.left_foot.x 	= robot_.left_foot()->motion_act().pos.x_vec[output_index_];
	output_.left_foot.y 	= robot_.left_foot()->motion_act().pos.y_vec[output_index_];
	output_.left_foot.z 	= robot_.left_foot()->motion_act().pos.z_vec[output_index_];
	output_.left_foot.yaw 	= robot_.left_foot()->motion_act().pos.yaw_vec[output_index_];
	output_.left_foot.dx 	= robot_.left_foot()->motion_act().vel.x_vec[output_index_];
	output_.left_foot.dy 	= robot_.left_foot()->motion_act().vel.y_vec[output_index_];
	output_.left_foot.dz 	= robot_.left_foot()->motion_act().vel.z_vec[output_index_];
	output_.left_foot.dyaw 	= robot_.left_foot()->motion_act().vel.yaw_vec[output_index_];
	output_.left_foot.ddx 	= robot_.left_foot()->motion_act().acc.x_vec[output_index_];
	output_.left_foot.ddy 	= robot_.left_foot()->motion_act().acc.y_vec[output_index_];
	output_.left_foot.ddz 	= robot_.left_foot()->motion_act().acc.z_vec[output_index_];
	output_.left_foot.ddyaw = robot_.left_foot()->motion_act().acc.yaw_vec[output_index_];

	output_.right_foot.x 		= robot_.right_foot()->motion_act().pos.x_vec[output_index_];
	output_.right_foot.y 		= robot_.right_foot()->motion_act().pos.y_vec[output_index_];
	output_.right_foot.z 		= robot_.right_foot()->motion_act().pos.z_vec[output_index_];
	output_.right_foot.yaw 		= robot_.right_foot()->motion_act().pos.yaw_vec[output_index_];
	output_.right_foot.dx 		= robot_.right_foot()->motion_act().vel.x_vec[output_index_];
	output_.right_foot.dy 		= robot_.right_foot()->motion_act().vel.y_vec[output_index_];
	output_.right_foot.dz 		= robot_.right_foot()->motion_act().vel.z_vec[output_index_];
	output_.right_foot.dyaw 	= robot_.right_foot()->motion_act().vel.yaw_vec[output_index_];
	output_.right_foot.ddx 		= robot_.right_foot()->motion_act().acc.x_vec[output_index_];
	output_.right_foot.ddy 		= robot_.right_foot()->motion_act().acc.y_vec[output_index_];
	output_.right_foot.ddz 		= robot_.right_foot()->motion_act().acc.z_vec[output_index_];
	output_.right_foot.ddyaw 	= robot_.right_foot()->motion_act().acc.yaw_vec[output_index_];
}

void Walkgen::SetPosReference(double x, double y){
	pos_ref_.global.x.fill(x);
	pos_ref_.global.y.fill(y);
}

void Walkgen::SetVelReference(double dx, double dy, double dyaw){//TODO: Is newVelRef_ necessary
	new_vel_ref_.local.x.fill(dx);
	new_vel_ref_.local.y.fill(dy);
	new_vel_ref_.local.yaw.fill(dyaw);
}

void Walkgen::SetVelReference(const CommonVectorType &x_vec, const CommonVectorType &y_vec, const CommonVectorType &yaw_vec){
	new_vel_ref_.local.x = x_vec;
	new_vel_ref_.local.y = y_vec;
	new_vel_ref_.local.yaw = yaw_vec;
}

void Walkgen::SetCPReference(double global_x, double global_y, double local_x, double local_y){
	cp_ref_.global.x.fill(global_x);
	cp_ref_.global.y.fill(global_y);
	cp_ref_.local.x.fill(local_x);
	cp_ref_.local.y.fill(local_y);
}

void Walkgen::StoreResult() {
	int num_steps_previewed = solution_.support_states_vec.back().step_number;
	int num_samples = mpc_parameters_.num_samples_contr;
	int num_unst_modes = 0;
	if (mpc_parameters_.formulation == DECOUPLED_MODES) {
		num_unst_modes = 1;
	}
	if (num_steps_previewed > 0) {
		solution_.prev_first_foot_x = solution_.qp_solution_vec(2*(num_samples + num_unst_modes));
		solution_.prev_first_foot_y = solution_.qp_solution_vec(2*(num_samples + num_unst_modes) + num_steps_previewed);
	}

	solution_.prev_cop_x = solution_.qp_solution_vec(0);
	solution_.prev_cop_y = solution_.qp_solution_vec(num_samples + num_unst_modes);

	solution_.is_prev_sol_exist = true;
}

void Walkgen::SetWalkingMode() {

	// Set vel reference:
	// ------------------
	if (robot_.current_support().phase == SS && robot_.current_support().num_steps_left == 0) {
		vel_ref_.local.x.fill(0);
		vel_ref_.local.y.fill(0);
		vel_ref_.local.yaw.fill(0);
	}

	// Set active walking mode:
	// ------------------------
	// TODO: Walking activated by velocity reference
	if (fabs(vel_ref_.local.yaw(0)) < kEps && fabs(vel_ref_.local.x(0)) < kEps && fabs(vel_ref_.local.y(0)) < kEps) {
		if (mpc_parameters_.walking_mode.type == WALK) {
			mpc_parameters_.penalties.active_mode 	= 0;//TODO: No set of penalties for stop after walking!
			mpc_parameters_.walking_mode.type 		= STOP;
			mpc_parameters_.walking_mode.start_time = current_time_;
		}
	} else {
		if (mpc_parameters_.walking_mode.type == INITIAL || mpc_parameters_.walking_mode.type == STOP) {
			mpc_parameters_.walking_mode.start_time = current_time_;
			mpc_parameters_.walking_mode.type		= WALK_START;

			mpc_parameters_.penalties.active_mode 		= 0;
			mpc_parameters_.penalties.is_initial_mode 	= false; //TODO: Unnecessary
		} else if (mpc_parameters_.walking_mode.type == WALK_START) {
			mpc_parameters_.walking_mode.type			= WALK;
		}
	}

	if (solution_.support_states_vec.front().phase == DS && mpc_parameters_.walking_mode.type == STOP) {
		double mid_feet_x = (robot_.left_foot()->state().x[0] + robot_.right_foot()->state().x[0]) / 2.;
		double mid_feet_y = (robot_.left_foot()->state().y[0] + robot_.right_foot()->state().y[0]) / 2.;
		pos_ref_.global.x.fill(mid_feet_x);
		pos_ref_.global.y.fill(mid_feet_y);
		cp_ref_.global.x.fill(mid_feet_x);
		cp_ref_.global.y.fill(mid_feet_y);
		vel_ref_.global.x.fill(0.);
		vel_ref_.global.y.fill(0.);

		mpc_parameters_.penalties.cop[1] = mpc_parameters_.penalties.cop[0];
		mpc_parameters_.penalties.contr_moves[1] = mpc_parameters_.penalties.contr_moves[0];
	}


}


