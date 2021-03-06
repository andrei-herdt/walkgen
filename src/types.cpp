#include <mpc-walkgen/types.h>

#include <mpc-walkgen/debug.h>

using namespace MPCWalkgen;

WalkingMode::WalkingMode()
:type(INITIAL),
 start_time(0.)
{}

Frame::Frame()
:x(1),y(1),yaw(1)
{
	x.fill(0);
	y.fill(0);
	yaw.fill(0);
}

void Frame::SetZero(int size){
	x.setZero(size);
	y.setZero(size);
	yaw.setZero(size);
}

Reference::Reference()
:global()
,local()
{}

void Reference::SetZero(int size){
	global.SetZero(size);
	local.SetZero(size);

	init.fill(0.);
	offset_ss.fill(0.);
	offset_ds.fill(0.);

	z_ref_x = 0.;
	z_ref_y = 0.;
}

BodyState::BodyState() {
	Reset();
}

void BodyState::Reset() {
	x.fill(0);
	y.fill(0);
	z.fill(0);
	yaw.fill(0);
	pitch.fill(0);
	roll.fill(0);
}

void Wrench::SetZero() {
	force_x = 0.;
	force_y = 0.;
	force_z = 0.;
	torque_x = 0.;
	torque_y = 0.;
	torque_z = 0.;
}

FootData::FootData()
: sole_width(0)
, sole_height(0)
, ankle_pos_local() {
	edges_x_vec.resize(4);
	edges_y_vec.resize(4);

	position.resize(3);
}

FootData::FootData(const FootData &f)
: sole_width(f.sole_width)
, sole_height(f.sole_height)
, ankle_pos_local(f.ankle_pos_local) {
	edges_x_vec = f.edges_x_vec;
	edges_y_vec = f.edges_y_vec;
	position = f.position;
}

FootData::~FootData(){
}

void FootData::SetEdges(double front, double back, double left, double right,
		double sec_marg_left, double sec_marg_right, double sec_marg_front, double sec_marg_back)
{
	// Clockwise, starting at front-left corner
	// Foots sagittal plane is aligned with the x-axis
	edges_x_vec[0] = front - sec_marg_front;
	edges_x_vec[1] = front - sec_marg_front;
	edges_x_vec[2] = back + sec_marg_back;
	edges_x_vec[3] = back + sec_marg_back;

	edges_y_vec[0] = left - sec_marg_left;
	edges_y_vec[1] = right + sec_marg_right;
	edges_y_vec[2] = right + sec_marg_right;
	edges_y_vec[3] = left - sec_marg_left;
}

HipYawData::HipYawData()
:lower_pos_bound(-0.523599)
,upper_pos_bound(0.785398)
,lower_vel_bound(-3.54108)
,upper_vel_bound(3.54108)
,lower_acc_bound(-0.1)
,upper_acc_bound(0.1) {}

HipYawData::~HipYawData() {}

SolutionAnalysis::SolutionAnalysis():num_iterations(0)
,objective_value(0)
,resolution_time(0)
,max_kkt_violation(0)
{}

MPCSolution::MPCSolution()
:is_prev_sol_exist(false)
,prev_first_foot_x(0.)
,prev_first_foot_y(0.)
,prev_cop_x(0.)
,prev_cop_y(0.)
,first_coarse_period(0.)
{}

MPCSolution::~MPCSolution() {}


MPCSolution& MPCSolution::operator = (MPCSolution const &rhs) {
	qp_solution_vec = rhs.qp_solution_vec;
	initial_solution = rhs.initial_solution;

	constraints = rhs.constraints;
	init_active_set = rhs.init_active_set;

	/// \brief Sampling times
	/// starting with 0, i.e. all times are relative to the current time
	sampling_times_vec = rhs.sampling_times_vec;

	support_states_vec = rhs.support_states_vec;

	support_yaw_vec = rhs.support_yaw_vec;//TODO: supportOrientations_vec
	trunk_yaw_vec = rhs.trunk_yaw_vec;//TODO: TrunkOrientations_vec

	com_prw = rhs.com_prw;
	left_foot_prw = rhs.left_foot_prw;
	right_foot_prw = rhs.right_foot_prw;

	com_act = rhs.com_act;
	left_foot_act = rhs.left_foot_act;
	right_foot_act = rhs.right_foot_act;

	analysis = rhs.analysis;

	pos_ref = rhs.pos_ref;

	is_prev_sol_exist = rhs.is_prev_sol_exist;

	prev_first_foot_x = rhs.prev_first_foot_x;
	prev_first_foot_y = rhs.prev_first_foot_y;
	prev_cop_x = rhs.prev_cop_x;
	prev_cop_y = rhs.prev_cop_y;


	return(*this);
}


void MPCSolution::Reset(){
	support_states_vec.resize(0);
	support_yaw_vec.resize(0);
	trunk_yaw_vec.resize(0);
}

void Trajectory::SetZero(int size) {
	x_vec.setZero(size);
	y_vec.setZero(size);
	z_vec.setZero(size);
	yaw_vec.setZero(size);
}

void Motion::SetZero(int num_samples, int num_unst_modes) {
	pos.SetZero(num_samples);
	vel.SetZero(num_samples);
	acc.SetZero(num_samples);
	jerk.SetZero(num_samples);
	control.SetZero(num_samples + num_unst_modes);
	cop.SetZero(num_samples);
	cp.SetZero(num_samples);
}

MPCParameters::MPCParameters()
:period_qpsample(0.)
,period_inter_samples(0.)
,period_recomputation(0.)
,period_actsample(0.)
,period_ds(1000000000.)
,period_dsss(0.8)
,period_ss(0.)
,ds_force_thresh(1000.)
,ffoot_plan_period(kInf)
,init_com_height(0.)
,cop_off(0.)
,num_samples_horizon_max(0)
,num_samples_contr(0)
,num_samples_state(0)
,num_samples_first_coarse_period(0)
,num_samples_first_fine_period(0)
,num_steps_ssds(0)
,num_steps_max(0)
,warmstart(false)
,interpolate_whole_horizon(false)
,is_closed_loop(false)
,is_pid_mode(false)
,is_ineq_constr(true)
,is_terminal_constr(false)
,problem_dumping(false)
,dynamics_order(THIRD_ORDER)
,formulation(STANDARD)
,mapping(ZERO_MAP)
,penalties(2)
{}

MPCParameters::~MPCParameters(){}

int MPCParameters::GetMPCSamplesLeft(double first_sampling_period) const {
	//TODO: Not good to have it here/ No good condition
	if (mapping == CONST_MAP) {
			return 0;
		} else {
			return static_cast<int> (round((period_qpsample - first_sampling_period) / period_recomputation));
		}

}

int MPCParameters::GetNumDynamics() const {
	//TODO: Not good to have it here/ No good condition
	if (mapping == CONST_MAP) {
		return 1;
	} else {
		return static_cast<int> (round(period_qpsample / period_recomputation));
	}
}

int MPCParameters::num_samples_act() const {
	return static_cast<int> (round(period_recomputation / period_actsample) );
}

double MPCParameters::period_trans_ds() const {
	return period_qpsample;
}

ConvexHull::ConvexHull()
:x_vec(1)
,y_vec(1)
,z_vec(1)
,a_vec(1)
,b_vec(1)
,c_vec(1)
,d_vec(1) {
}

ConvexHull::~ConvexHull() {}

// TODO: Necessary?
ConvexHull &ConvexHull::operator=(const ConvexHull &hull){
	x_vec = hull.x_vec;
	y_vec = hull.y_vec;
	z_vec = hull.z_vec;
	a_vec = hull.a_vec;
	b_vec = hull.b_vec;
	c_vec = hull.c_vec;
	d_vec = hull.d_vec;

	return *this;
}

void ConvexHull::Resize(int size) {
	if (size != x_vec.rows()) {
		x_vec.setZero(size);
		y_vec.setZero(size);
		a_vec.setZero(size);
		b_vec.setZero(size);
		c_vec.setZero(size);
		d_vec.setZero(size);
	} else {
		x_vec.fill(0);
		y_vec.fill(0);
		a_vec.fill(0);
		b_vec.fill(0);
		c_vec.fill(0);
		d_vec.fill(0);
	}
}

void ConvexHull::RotateVertices(double yaw) {
	if (fabs(yaw) < kEps)
		return;

	double x_old, y_old;
	int size = x_vec.rows();
	for(int i = 0; i < size; ++i){
		x_old = x_vec(i);
		y_old = y_vec(i);
		x_vec(i) = (x_old * std::cos(yaw) - y_old * std::sin(yaw));
		y_vec(i) = (x_old * std::sin(yaw) + y_old * std::cos(yaw));
	}
}

void ConvexHull::BuildInequalities(const Foot &foot) {
	double dx,dy,dc,x1,y1,x2,y2;
	unsigned nbRows = x_vec.rows();

	double sign;
	if(foot == LEFT){
		sign = 1.0;
	}else{
		sign = -1.0;
	}
	for( unsigned i=0; i<nbRows;++i ){
		y1 = y_vec(i);
		y2 = y_vec((i+1)%nbRows);
		x1 = x_vec(i);
		x2 = x_vec((i+1)%nbRows);

		dx = sign*(y1-y2);
		dy = sign*(x2-x1);
		dc = dx*x1+dy*y1;


		a_vec(i) = dx;
		b_vec(i) = dy;
		d_vec(i) = dc;
	}
}

RobotData::RobotData(const FootData &leftFoot, const FootData &rightFoot,
		const HipYawData &left_hip_yaw, const HipYawData &rightHipYaw,
		double mass)
:mass(mass)
,max_foot_height(0.03)
,max_foot_vel(0.)
,security_margin(-1.)
,lateral_ds_feet_dist(-1.)
,com()
,left_foot(leftFoot)
,right_foot(rightFoot)
,left_hip_yaw(left_hip_yaw)
,right_hip_yaw(rightHipYaw)
,left_foot_pos_hull()
,right_foot_pos_hull()
,left_foot_ss_hull()
,right_foot_ss_hull()
,left_foot_ds_hull()
,right_foot_ds_hull() {
	left_foot_ss_hull.Resize(4);
	right_foot_ss_hull.Resize(4);
	left_foot_ds_hull.Resize(4);
	right_foot_ds_hull.Resize(4);
}

RobotData::RobotData():mass(0.)
,max_foot_height(0.)
,max_foot_vel(0.)
,security_margin(-1.)
,lateral_ds_feet_dist(-1.) {
	left_foot_ss_hull.Resize(4);
	right_foot_ss_hull.Resize(4);
	left_foot_ds_hull.Resize(4);
	right_foot_ds_hull.Resize(4);
}
RobotData::~RobotData(){}

// TODO: This function should be moved to a separate object for hulls/constraints
void RobotData::SetCoPHulls(double ds_distance) {
	for (int i = 0; i < 4; ++i) {
		left_foot_ss_hull.x_vec(i) = left_foot.edges_x_vec[i];
		left_foot_ss_hull.y_vec(i) = left_foot.edges_y_vec[i];
		left_foot_ds_hull.x_vec(i) = left_foot.edges_x_vec[i];
		left_foot_ds_hull.y_vec(i) = left_foot.edges_y_vec[i];

		right_foot_ss_hull.x_vec(i) = right_foot.edges_x_vec[i];
		right_foot_ss_hull.y_vec(i) = right_foot.edges_y_vec[i];
		right_foot_ds_hull.x_vec(i) = right_foot.edges_x_vec[i];
		right_foot_ds_hull.y_vec(i) = right_foot.edges_y_vec[i];
	}
	left_foot_ds_hull.y_vec(1)  -= ds_distance;
	left_foot_ds_hull.y_vec(2)  -= ds_distance;
	right_foot_ds_hull.y_vec(1) += ds_distance;
	right_foot_ds_hull.y_vec(2) += ds_distance;

	lateral_ds_feet_dist = ds_distance;

}

Penalties::Penalties(int num_modes)
:pos(num_modes)
,vel(num_modes)
,acc(num_modes)
,jerk(num_modes)
,cop(num_modes)
,cp(num_modes)
,cp_fp(num_modes)
,contr_moves(num_modes)
,first_contr_moves(0.)
,second_contr_moves(0.)
,active_mode(1)
,is_initial_mode(true)
,dcop_online(false)
,cop_online(false)
{
	pos[0] 			= 0.;
	vel[0]  		= 0.;
	cop[0]  		= 0.;
	cp[0] 			= 0.;
	cp_fp[0]		= 0.;
	contr_moves[0] 	= 0.;

	pos[1] 			= 0.;
	vel[1]  		= 0.;
	cop[1]		  	= 0.;
	cp[1] 			= 0.;
	cp_fp[1]		= 0.;
	contr_moves[1] 	= 0.;
}
Penalties::~Penalties(){}

SupportState::SupportState()
:phase(DS)
,foot(LEFT)
,num_steps_left(0)
,step_number(0)
,num_instants(0)
,time_limit(0.)
,start_time(0.)
,x(0.)
,y(0.)
,yaw(0.)
,state_changed(false)
,transitional_ds(false)
,is_half_ds_passed(false)
,previous_sampling_period(0.)
{}

void Penalties::SetCoefficients(Reference &ref) {
	if (fabs(ref.local.yaw(0)) < kEps && fabs(ref.local.x(0)) < kEps && fabs(ref.local.y(0)) < kEps) {
		if (is_initial_mode) {
			active_mode = 1;
		} else {
			active_mode = 1;
		}
	} else {
		active_mode = 0;
		is_initial_mode = false;
	}
}

void LinearDynamicsMatrices::SetZero(int state_dim, int input_dim, int output_dim, int num_samples, int stable_dim, int unstable_dim) {
	state_mat.setZero(num_samples, stable_dim);
	//stab_state_mat.setZero(num_samples, stable_dim);
	//unst_state_mat.setZero(num_samples, unstable_dim);
	input_mat.setZero(num_samples, num_samples + unstable_dim);
	input_mat_tr.setZero(num_samples + unstable_dim, num_samples);
	input_mat_inv.setZero(num_samples, num_samples);
	input_mat_inv_tr.setZero(num_samples, num_samples);

	c_state_mat.setZero(state_dim, state_dim); 	c_state_mat_inv.setZero(state_dim, state_dim);
	d_state_mat.setZero(state_dim, state_dim); 	d_state_mat_inv.setZero(state_dim, state_dim);
	c_input_mat.setZero(state_dim, input_dim); 	c_input_mat_tr.setZero(input_dim, state_dim);
	d_input_mat.setZero(state_dim, input_dim); 	d_input_mat_tr.setZero(input_dim, state_dim);
	ss_output_mat.setZero(output_dim, state_dim); 	ss_output_mat_tr.setZero(state_dim, input_dim);
	ss_feedthrough_mat.setZero(output_dim, input_dim);
}

void LinearDynamics::SetZero(int state_dim, int input_dim, int output_dim, int num_samples_state, int num_samples_contr, int stable_dim, int unstable_dim) {
	pos.SetZero(state_dim, input_dim, output_dim, num_samples_state, stable_dim, unstable_dim);
	vel.SetZero(state_dim, input_dim, output_dim, num_samples_state, stable_dim, unstable_dim);
	acc.SetZero(state_dim, input_dim, output_dim, num_samples_state, stable_dim, unstable_dim);
	jerk.SetZero(state_dim, input_dim, output_dim, num_samples_state, stable_dim, unstable_dim);
	cop.SetZero(state_dim, input_dim, output_dim, num_samples_state, stable_dim, unstable_dim);
	cp.SetZero(state_dim, input_dim, output_dim, num_samples_state, stable_dim, unstable_dim);

	cont_ss.SetZero(state_dim, input_dim, output_dim, num_samples_state, stable_dim, unstable_dim);
	discr_ss.SetZero(state_dim, input_dim, output_dim, num_samples_state, stable_dim, unstable_dim);

	d_state_mat_vec.resize(num_samples_state, CommonMatrixType::Zero(state_dim, state_dim));
	d_state_mat_pow_vec.resize(num_samples_state, CommonMatrixType::Zero(state_dim, state_dim));
	rev_matrix_prod_vec.resize(num_samples_state, CommonMatrixType::Zero(state_dim, state_dim));
	d_state_mat_pow2_vec.resize(num_samples_state, CommonMatrixType::Zero(state_dim, state_dim));
	d_input_mat_vec.resize(num_samples_state, CommonMatrixType::Zero(state_dim, input_dim));

	input_map_mat.setZero(num_samples_state, num_samples_contr);
	input_map_mat_tr.setZero(num_samples_contr, num_samples_state);
}


SelectionMatrices::SelectionMatrices(int num_rows)
:sample_step(num_rows, num_rows)
,sample_step_trans(num_rows, num_rows)
,sample_step_cx(num_rows)
,sample_step_cy(num_rows)
,Vf(num_rows, num_rows)
,VcfX(num_rows)
,VcfY(num_rows)
{}

void SelectionMatrices::SetZero() {
	sample_step.setZero();
	sample_step_trans.setZero();
	sample_step_cx.setZero();
	sample_step_cy.setZero();
	Vf.setZero();
	VcfX.setZero();
	VcfY.setZero();
}

void RelativeInequalities::Resize(int rows, int cols){
	if (rows != x_mat.rows() || cols != x_mat.cols()){
		x_mat.setZero(rows, cols);
		y_mat.setZero(rows, cols);
		c_vec.setZero(rows, cols);
	}else{
		x_mat.fill(0);
		y_mat.fill(0);
		c_vec.fill(0);
	}

}
