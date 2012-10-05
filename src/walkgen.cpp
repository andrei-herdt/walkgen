#include <mpc-walkgen/walkgen.h>

using namespace MPCWalkgen;

Walkgen::Walkgen()
: mpc_parameters_()
,robot_data_()
,solver_(NULL)
,builder_(NULL)
,preview_(NULL)
,robot_(NULL)
,orient_preview_(NULL)
,clock_()
,output_()
,output_index_(0)
,solution_()
,vel_ref_()
,first_sample_time_(0)
,next_computation_(0)
,next_act_sample_(0)
,current_time_(0) {
	orient_preview_ = new OrientationsPreview();

	robot_ = new RigidBodySystem();

	clock_.ReserveMemory(20);
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

	if (preview_ != NULL) {
		delete preview_;
		preview_ = NULL;
	}

	if (robot_ != NULL) {
		delete robot_;
		robot_ = NULL;
	}
}

void Walkgen::Init(const MPCParameters &mpc_parameters) {
	mpc_parameters_ = mpc_parameters;

	robot_->Init(&mpc_parameters_);

	// Solver:
	// -------
	int num_constr_step = 5;// TODO: Move this to MPCParameters
	int num_steps_max = mpc_parameters_.num_steps_max();
	int num_vars_max = 2 * (mpc_parameters_.num_samples_horizon + num_steps_max);
	int num_constr_max = num_constr_step * num_steps_max;
	solver_ = createQPSolver(mpc_parameters_.solver, num_vars_max,  num_constr_max );

	// Set order of optimization variables
	Eigen::VectorXi order(solver_->nbvar_max());
	for (int i = 0; i < mpc_parameters_.num_samples_horizon; ++i) {// 0,2,4,1,3,5 (CoM)
		order(i) = 2 * i;
		order(i + mpc_parameters_.num_samples_horizon) = 2*i+1;
	}
	for (int i = 2 * mpc_parameters_.num_samples_horizon; i < solver_->nbvar_max(); ++i) {// 6,7,8 (Feet)
		order(i) = i;
	}
	solver_->SetVarOrder(order);

	// Resize:
	// -------
	solution_.com_act.resize(mpc_parameters_.num_samples_act());
	solution_.cop_act.resize(mpc_parameters_.num_samples_act());
	solution_.com_prw.resize(mpc_parameters_.num_samples_horizon);
	solution_.cop_prw.resize(mpc_parameters_.num_samples_horizon);

	solution_.pos_ref.resize(mpc_parameters_.num_samples_horizon);

	vel_ref_.resize(mpc_parameters_.num_samples_horizon);
	new_vel_ref_.resize(mpc_parameters_.num_samples_horizon);

	// Reset:
	// ------
	ResetCounters(0.0);
}

void Walkgen::Init(const RobotData &robot_data) {
	robot_data_ = robot_data;
	robot_->Init(robot_data_);
	Init();

}

const MPCSolution &Walkgen::Go(){
	current_time_ += mpc_parameters_.period_mpcsample;
	return Go(current_time_);
}

const MPCSolution &Walkgen::Go(double time){
	current_time_ = time;

	if (time  > next_computation_ - kEps) {
		//int first_timer = clock_.StartCounter();
		next_computation_ += mpc_parameters_.period_mpcsample;
		if (time > next_computation_ - kEps) {
			ResetCounters(time);
		}
		if(time  > first_sample_time_ - kEps){
			first_sample_time_ += mpc_parameters_.period_qpsample;
			if (time > first_sample_time_ - kEps) {
				ResetCounters(time);
			}
		}
		ResetOutputIndex();
		builder_->current_time(current_time_);
		//clock_.StopCounter(first_timer);

		int timer_build_problem = clock_.StartCounter();
		BuildProblem();
		clock_.StopCounter(timer_build_problem);

		int timer_solve = clock_.StartCounter();
		solver_->Solve(solution_, mpc_parameters_.warmstart, mpc_parameters_.solver.analysis);
		clock_.StopCounter(timer_solve);

		int timer_generate_traj = clock_.StartCounter();
		GenerateTrajectories();
		clock_.StopCounter(timer_generate_traj);
	}

	//int timer_update_output = clock_.StartCounter();
	if (time > next_act_sample_ - kEps) {
		next_act_sample_ += mpc_parameters_.period_actsample;

		IncrementOutputIndex();
		UpdateOutput();
	}
	//clock_.StopCounter(timer_update_output);

	//clock_.StartCounter();

	return solution_;
}

//
// Private methods:
//
void Walkgen::Init() {
	robot_->ComputeDynamics();

	preview_ = new HeuristicPreview(&vel_ref_, robot_, &mpc_parameters_, &clock_);

	builder_= new QPBuilder(preview_, solver_, &vel_ref_, &weight_coefficients_, robot_, &mpc_parameters_, &clock_);

	orient_preview_->Init(mpc_parameters_, robot_data_);

	builder_->PrecomputeObjective();

	BodyState left_foot_state;
	left_foot_state.x[0] = robot_data_.leftFoot.position[0];
	left_foot_state.y[0] = robot_data_.leftFoot.position[1];
	robot_->body(LEFT_FOOT)->state(left_foot_state);

	BodyState right_foot_state;
	right_foot_state.x[0] = robot_data_.right_foot.position[0];
	right_foot_state.y[0] = robot_data_.right_foot.position[1];
	robot_->body(RIGHT_FOOT)->state(right_foot_state);

	BodyState state_com;
	state_com.x[0] = robot_data_.com(0);//TODO: Add macros for x,y,z
	state_com.y[0] = robot_data_.com(1);
	state_com.z[0] = robot_data_.com(2);
	robot_->body(COM)->state(state_com);

	weight_coefficients_.active_mode = 0;

	BuildProblem();

	solver_->Init();
}

void Walkgen::SetCounters(double time) {}

void Walkgen::ResetCounters(double time) {
	first_sample_time_ = time + mpc_parameters_.period_qpsample;
	next_computation_ = time + mpc_parameters_.period_mpcsample;
	next_act_sample_ = time + mpc_parameters_.period_actsample;
}

void Walkgen::BuildProblem() {
	// UPDATE INTERNAL DATA:
	// ---------------------
	solver_->reset();
	solution_.reset();
	vel_ref_ = new_vel_ref_;

	if (robot_->current_support().phase == SS && robot_->current_support().num_steps_left == 0) {
		vel_ref_.local.x.fill(0);
		vel_ref_.local.y.fill(0);
		vel_ref_.local.yaw.fill(0);
	}
	weight_coefficients_.SetCoefficients(vel_ref_);

	double first_sampling_period = first_sample_time_ - current_time_;
	robot_->setSelectionNumber(first_sampling_period);

	// PREVIEW:
	// --------
	preview_->PreviewSamplingTimes(current_time_, first_sampling_period, solution_);

	preview_->PreviewSupportStates(first_sampling_period, solution_);

	orient_preview_->preview_orientations( current_time_, vel_ref_,
			mpc_parameters_.nbqpsamples_step * mpc_parameters_.period_qpsample,
			robot_->body(LEFT_FOOT)->state(), robot_->body(RIGHT_FOOT)->state(),
			solution_ );
	preview_->BuildRotationMatrix(solution_);

	builder_->BuildReferenceVector(solution_);

	builder_->BuildProblem(solution_);
}

void Walkgen::GenerateTrajectories() {
	builder_->TransformControlVector(solution_);	//Half of the time spent here. Can be reduced.

	robot_->Interpolate(solution_, current_time_, vel_ref_);

	robot_->UpdateState(solution_);

	orient_preview_->InterpolateTrunkYaw(robot_);

	ResetOutputIndex();
	UpdateOutput();
}


void Walkgen::ResetOutputIndex() {
	output_index_ = 0;
}

void Walkgen::IncrementOutputIndex() {
	if (output_index_ < mpc_parameters_.num_samples_act() - 1) {
		output_index_++;
	}
}

void Walkgen::UpdateOutput() {
	output_.com.x = solution_.com_act.pos.x_vec[output_index_];
	output_.com.y = solution_.com_act.pos.y_vec[output_index_];
	output_.com.z = robot_->body(COM)->state().z(0);
	output_.com.dx = solution_.com_act.vel.x_vec[output_index_];
	output_.com.dy = solution_.com_act.vel.y_vec[output_index_];
	output_.com.dz = robot_->body(COM)->state().z(1);
	output_.com.ddx = solution_.com_act.acc.x_vec[output_index_];
	output_.com.ddy = solution_.com_act.acc.y_vec[output_index_];
	output_.com.ddz = robot_->body(COM)->state().z(2);

	output_.cop.x =   solution_.cop_act.pos.x_vec[output_index_];
	output_.cop.y =   solution_.cop_act.pos.y_vec[output_index_];

	output_.left_foot.x = robot_->left_foot()->motion_act().pos.x_vec[output_index_];
	output_.left_foot.y = robot_->left_foot()->motion_act().pos.y_vec[output_index_];
	output_.left_foot.z = robot_->left_foot()->motion_act().pos.z_vec[output_index_];
	output_.left_foot.yaw = robot_->left_foot()->motion_act().pos.yaw_vec[output_index_];
	output_.left_foot.dx = robot_->left_foot()->motion_act().vel.x_vec[output_index_];
	output_.left_foot.dy = robot_->left_foot()->motion_act().vel.y_vec[output_index_];
	output_.left_foot.dz = robot_->left_foot()->motion_act().vel.z_vec[output_index_];
	output_.left_foot.dyaw = robot_->left_foot()->motion_act().vel.yaw_vec[output_index_];
	output_.left_foot.ddx = robot_->left_foot()->motion_act().acc.x_vec[output_index_];
	output_.left_foot.ddy = robot_->left_foot()->motion_act().acc.y_vec[output_index_];
	output_.left_foot.ddz = robot_->left_foot()->motion_act().acc.z_vec[output_index_];
	output_.left_foot.ddyaw = robot_->left_foot()->motion_act().acc.yaw_vec[output_index_];

	output_.right_foot.x = robot_->right_foot()->motion_act().pos.x_vec[output_index_];
	output_.right_foot.y = robot_->right_foot()->motion_act().pos.y_vec[output_index_];
	output_.right_foot.z = robot_->right_foot()->motion_act().pos.z_vec[output_index_];
	output_.right_foot.yaw = robot_->right_foot()->motion_act().pos.yaw_vec[output_index_];
	output_.right_foot.dx = robot_->right_foot()->motion_act().vel.x_vec[output_index_];
	output_.right_foot.dy = robot_->right_foot()->motion_act().vel.y_vec[output_index_];
	output_.right_foot.dz = robot_->right_foot()->motion_act().vel.z_vec[output_index_];
	output_.right_foot.dyaw = robot_->right_foot()->motion_act().vel.yaw_vec[output_index_];
	output_.right_foot.ddx = robot_->right_foot()->motion_act().acc.x_vec[output_index_];
	output_.right_foot.ddy = robot_->right_foot()->motion_act().acc.y_vec[output_index_];
	output_.right_foot.ddz = robot_->right_foot()->motion_act().acc.z_vec[output_index_];
	output_.right_foot.ddyaw = robot_->right_foot()->motion_act().acc.yaw_vec[output_index_];
}

void Walkgen::SetReference(double dx, double dy, double dyaw){//TODO: Is newVelRef_ necessary
	new_vel_ref_.local.x.fill(dx);
	new_vel_ref_.local.y.fill(dy);
	new_vel_ref_.local.yaw.fill(dyaw);
}

void Walkgen::SetReference(CommonVectorType dx, CommonVectorType dy, CommonVectorType dyaw){
	new_vel_ref_.local.x = dx;
	new_vel_ref_.local.y = dy;
	new_vel_ref_.local.yaw = dyaw;
}


