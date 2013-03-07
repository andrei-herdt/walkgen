#include <mpc-walkgen.h>

#ifdef __cplusplus
extern "C" {
#endif

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  walkgen_sfun

#include "simstruc.h"

using namespace Eigen;
using namespace MPCWalkgen;
using namespace std;

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
		!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

static void mdlInitializeSizes(SimStruct *S) {
	// Expected number of parameters
	ssSetNumSFcnParams(S, 31);

	// Parameter mismatch?
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
		return;
	}

	// Specify I/O
	if (!ssSetNumInputPorts(S, 13)) return;
	ssSetInputPortWidth(S, 0, 2);     //pos_ref
	ssSetInputPortWidth(S, 1, 3);     //vel_ref
	ssSetInputPortWidth(S, 2, 4);     //cp_ref (Global reference and local offset)
	ssSetInputPortWidth(S, 3, 3);     //left_ankle_in
	ssSetInputPortWidth(S, 4, 3);     //right_ankle_in
	ssSetInputPortWidth(S, 5, 1);     //left_yaw
	ssSetInputPortWidth(S, 6, 1);     //right_yaw
	ssSetInputPortWidth(S, 7, 4);     //foot_geometry
	ssSetInputPortWidth(S, 8, 6);     //com_in
	ssSetInputPortWidth(S, 9, 2);     //cop
	ssSetInputPortWidth(S, 10, 6);    //lfoot_wrench_in
	ssSetInputPortWidth(S, 11, 6);    //rfoot_wrench_in
	ssSetInputPortWidth(S, 12, 1);    //reset_in

	ssSetInputPortDirectFeedThrough(S, 0, 1);
	ssSetInputPortDirectFeedThrough(S, 1, 1);
	ssSetInputPortDirectFeedThrough(S, 2, 1);
	ssSetInputPortDirectFeedThrough(S, 3, 1);
	ssSetInputPortDirectFeedThrough(S, 4, 1);
	ssSetInputPortDirectFeedThrough(S, 5, 1);
	ssSetInputPortDirectFeedThrough(S, 6, 1);
	ssSetInputPortDirectFeedThrough(S, 7, 1);
	ssSetInputPortDirectFeedThrough(S, 8, 1);
	ssSetInputPortDirectFeedThrough(S, 9, 1);
	ssSetInputPortDirectFeedThrough(S, 10, 1);
	ssSetInputPortDirectFeedThrough(S, 11, 1);
	ssSetInputPortDirectFeedThrough(S, 12, 1);

	if (!ssSetNumOutputPorts(S,19)) return;
	// Realized motions
	ssSetOutputPortWidth(S, 0, 3);        //com
	ssSetOutputPortWidth(S, 1, 3);        //dcom
	ssSetOutputPortWidth(S, 2, 3);        //ddcom
	ssSetOutputPortWidth(S, 3, 2);        //cop
	ssSetOutputPortWidth(S, 4, 4);        //p_left
	ssSetOutputPortWidth(S, 5, 4);        //dp_left
	ssSetOutputPortWidth(S, 6, 4);        //ddp_left
	ssSetOutputPortWidth(S, 7, 4);        //p_right
	ssSetOutputPortWidth(S, 8, 4);        //dp_right
	ssSetOutputPortWidth(S, 9, 4);        //ddp_right
	// Previewed motions:
	const int num_samples_horizon_max      = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 0)));
	const int kNumSamplesStep         = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 1)));
	const int num_steps_max           = 3;//kNumSamplesHorizon / kNumSamplesStep + 1;
	ssSetOutputPortWidth(S, 10, num_steps_max);               //first_foot_prw
	ssSetOutputPortWidth(S, 11, 4 * num_samples_horizon_max);     //com_prw (sample_instants, x, y, z)
	ssSetOutputPortWidth(S, 12, 3 * num_samples_horizon_max);     //cop_prw (sample_instants, x, y)
	ssSetOutputPortWidth(S, 13, 3);       //support
	ssSetOutputPortWidth(S, 14, 30);      //analysis
	ssSetOutputPortWidth(S, 15, 3 * num_samples_horizon_max);	//com_control_prw
	ssSetOutputPortWidth(S, 16, 3 * num_samples_horizon_max);     //cp_prw (sample_instants, x, y)
	ssSetOutputPortWidth(S, 17, 9);     //cp_prw (sample_instants, x, y)
	ssSetOutputPortWidth(S, 18, 7);     //sim_parameters (sample_instants, x, y)


	ssSetNumSampleTimes(S, 1);

	ssSetNumPWork(S, 1);
	ssSetNumIWork(S, 1);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
	const double kSamplingTime = *mxGetPr(ssGetSFcnParam(S, 6));
	ssSetSampleTime(S, 0, kSamplingTime);
}

#define MDL_START
static void mdlStart(SimStruct *S) {
	int is_debug_in 			= static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 10)));
	int is_pid_mode_in 			= static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 16)));
	int is_constraints_in 		= static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 18)));
	int formulation_in 			= static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 19)));
	int is_terminal_constr_in 	= static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 21)));
	int dump_problems_in 		= static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 20)));
	int solver_in 				= static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 22)));

	MPCParameters mpc_parameters;
	mpc_parameters.num_samples_horizon_max  		= static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 0)));
	mpc_parameters.num_samples_first_fine_period 	= static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 30)));
	mpc_parameters.num_samples_first_coarse_period	= static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 28)));
	mpc_parameters.period_ss     					= *mxGetPr(ssGetSFcnParam(S, 1));
	mpc_parameters.num_samples_dsss     			= min(static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 2))), mpc_parameters.num_samples_horizon_max);
	mpc_parameters.num_steps_ssds       			= min(static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 3))), mpc_parameters.num_samples_horizon_max);
	mpc_parameters.num_steps_max					= 3;
	mpc_parameters.period_qpsample     				= *mxGetPr(ssGetSFcnParam(S, 4));
	mpc_parameters.period_recomputation  	   		= *mxGetPr(ssGetSFcnParam(S, 5));
	mpc_parameters.period_actsample     			= *mxGetPr(ssGetSFcnParam(S, 6));
	mpc_parameters.period_inter_samples				= *mxGetPr(ssGetSFcnParam(S, 29));

	mpc_parameters.solver.num_wsrec     			= static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 8)));
	mpc_parameters.warmstart						= false;
	if (is_debug_in == 0) {
		mpc_parameters.interpolate_whole_horizon	= false;
		mpc_parameters.solver.analysis			    = false;
	} else {
		mpc_parameters.interpolate_whole_horizon	= true;
		mpc_parameters.solver.analysis			    = true;
	}
	if (solver_in == 0) {
		mpc_parameters.solver.name	= QLD;
	} else if (solver_in == 1) {
		mpc_parameters.solver.name  = QPOASES;
	} else if (solver_in == 2) {
		mpc_parameters.solver.name  = LSSOL;
	}


	mpc_parameters.dynamics_order               = static_cast<SystemOrder>(static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 9))));

	mpc_parameters.penalties.pos[0] 			= *mxGetPr(ssGetSFcnParam(S, 11));
	mpc_parameters.penalties.vel[0]  			= *mxGetPr(ssGetSFcnParam(S, 12));
	mpc_parameters.penalties.cop[0]  			= *mxGetPr(ssGetSFcnParam(S, 13));
	mpc_parameters.penalties.cp[0] 				= *mxGetPr(ssGetSFcnParam(S, 14));
	mpc_parameters.penalties.cp_fp[0]			= *mxGetPr(ssGetSFcnParam(S, 27));
	mpc_parameters.penalties.contr_moves[0] 	= *mxGetPr(ssGetSFcnParam(S, 15));
	mpc_parameters.penalties.first_contr_move	= *mxGetPr(ssGetSFcnParam(S, 23));

	mpc_parameters.penalties.pos[1] 			= mxGetPr(ssGetSFcnParam(S, 11))[1];
	mpc_parameters.penalties.vel[1]  			= mxGetPr(ssGetSFcnParam(S, 12))[1];
	mpc_parameters.penalties.cop[1]  			= mxGetPr(ssGetSFcnParam(S, 13))[1];
	mpc_parameters.penalties.cp[1] 				= mxGetPr(ssGetSFcnParam(S, 14))[1];
	mpc_parameters.penalties.cp_fp[1]			= mxGetPr(ssGetSFcnParam(S, 27))[1];
	mpc_parameters.penalties.contr_moves[1] 	= mxGetPr(ssGetSFcnParam(S, 15))[1];

	mpc_parameters.ds_force_thresh				= *mxGetPr(ssGetSFcnParam(S, 24));
	mpc_parameters.ffoot_plan_period 			= *mxGetPr(ssGetSFcnParam(S, 25));

	mpc_parameters.init_com_height				= *mxGetPr(ssGetSFcnParam(S, 26));

	if (is_pid_mode_in == 1) {
		mpc_parameters.is_pid_mode 			= true;
	}
	if (is_constraints_in == 0) {
		mpc_parameters.is_ineq_constr 		= false;
	}
	if (is_terminal_constr_in == 1) {
		mpc_parameters.is_terminal_constr 	= true;
	}
	if (formulation_in == 0) {
		mpc_parameters.formulation 			= STANDARD;
	} else if (formulation_in == 1) {
		mpc_parameters.formulation 			= DECOUPLED_MODES;
	}

	if (dump_problems_in == 1) {
		mpc_parameters.problem_dumping 		= true;
	}

	Walkgen *walk = new Walkgen;
	if (is_debug_in) {
		walk->clock().ReserveMemory(20, 2000);
		walk->clock().GetFrequency(100);
	}
	walk->clock().ReserveMemory(20, 3000);
	walk->clock().GetFrequency(100);

	walk->Init(mpc_parameters);

	ssSetPWorkValue(S, 0, (void*)walk);
	ssSetIWorkValue(S, 0, 0);     //MPCWalkgen not initialized

}

static void mdlOutputs(SimStruct *S, int_T tid) {
	const double kSecurityMargin    = *mxGetPr(ssGetSFcnParam(S, 7));
	const double kMaxFootHeight     = 0.03;
	const static int kDebug         = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 10)));
	int is_closed_loop_in           = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 17)));

	InputRealPtrsType pos_ref         = ssGetInputPortRealSignalPtrs(S, 0);
	InputRealPtrsType vel_ref         = ssGetInputPortRealSignalPtrs(S, 1);
	InputRealPtrsType cp_ref          = ssGetInputPortRealSignalPtrs(S, 2);
	InputRealPtrsType left_ankle_in   = ssGetInputPortRealSignalPtrs(S, 3);
	InputRealPtrsType right_ankle_in  = ssGetInputPortRealSignalPtrs(S, 4);
	InputRealPtrsType left_yaw        = ssGetInputPortRealSignalPtrs(S, 5);
	InputRealPtrsType right_yaw       = ssGetInputPortRealSignalPtrs(S, 6);
	InputRealPtrsType foot_geometry   = ssGetInputPortRealSignalPtrs(S, 7);
	InputRealPtrsType com_in          = ssGetInputPortRealSignalPtrs(S, 8);
	InputRealPtrsType cop_in          = ssGetInputPortRealSignalPtrs(S, 9);
	InputRealPtrsType lfoot_wrench_in = ssGetInputPortRealSignalPtrs(S, 10);
	InputRealPtrsType rfoot_wrench_in = ssGetInputPortRealSignalPtrs(S, 11);
	InputRealPtrsType reset_in        = ssGetInputPortRealSignalPtrs(S, 12);

	real_T *com            = ssGetOutputPortRealSignal(S, 0);
	real_T *dcom           = ssGetOutputPortRealSignal(S, 1);
	real_T *ddcom          = ssGetOutputPortRealSignal(S, 2);
	real_T *cop            = ssGetOutputPortRealSignal(S, 3);
	real_T *p_left         = ssGetOutputPortRealSignal(S, 4);
	real_T *dp_left        = ssGetOutputPortRealSignal(S, 5);
	real_T *ddp_left       = ssGetOutputPortRealSignal(S, 6);
	real_T *p_right        = ssGetOutputPortRealSignal(S, 7);
	real_T *dp_right       = ssGetOutputPortRealSignal(S, 8);
	real_T *ddp_right      = ssGetOutputPortRealSignal(S, 9);
	real_T *first_foot_prw = ssGetOutputPortRealSignal(S, 10);
	real_T *com_prw        = ssGetOutputPortRealSignal(S, 11);
	real_T *cop_prw        = ssGetOutputPortRealSignal(S, 12);
	real_T *support        = ssGetOutputPortRealSignal(S, 13);
	real_T *analysis       = ssGetOutputPortRealSignal(S, 14);
	real_T *com_control_prw= ssGetOutputPortRealSignal(S, 15);
	real_T *cp_prw         = ssGetOutputPortRealSignal(S, 16);
	real_T *cur_state      = ssGetOutputPortRealSignal(S, 17);
	real_T *sim_parameters = ssGetOutputPortRealSignal(S, 18);

	Walkgen *walk = (Walkgen *)ssGetPWorkValue(S, 0);

	// Begin initialization of the robot:
	// ----------------------------------
	if (ssGetIWorkValue(S, 0) == 0) {
		FootData left_foot;
		left_foot.ankle_pos_local 	<< -0.035, 0., 0.;
		left_foot.sole_height 		= *foot_geometry[2] - *foot_geometry[3];
		left_foot.sole_width 		= *foot_geometry[0]  - *foot_geometry[1];
		left_foot.SetEdges(*foot_geometry[0] + left_foot.ankle_pos_local(0), *foot_geometry[1] + left_foot.ankle_pos_local(0), *foot_geometry[2], *foot_geometry[3], kSecurityMargin);
		FootData right_foot;
		right_foot.ankle_pos_local 	<< -0.035, 0., 0.;
		right_foot.sole_height 		= *foot_geometry[2] - *foot_geometry[3];
		right_foot.sole_width 		= *foot_geometry[0] - *foot_geometry[1];
		right_foot.SetEdges(*foot_geometry[0] + right_foot.ankle_pos_local(0), *foot_geometry[1] + right_foot.ankle_pos_local(0), *foot_geometry[2], *foot_geometry[3], kSecurityMargin);

		HipYawData left_hip_yaw;
		left_hip_yaw.lower_pos_bound 	= -0.523599;
		left_hip_yaw.upper_pos_bound    = 0.785398;
		left_hip_yaw.lower_vel_bound    = -3.54108;
		left_hip_yaw.upper_vel_bound    = 3.54108;
		left_hip_yaw.lower_acc_bound    = -0.1;
		left_hip_yaw.upper_acc_bound    = 0.1;
		HipYawData right_hip_yaw        = left_hip_yaw;

		RobotData robot_data(left_foot, right_foot, left_hip_yaw, right_hip_yaw, 0.0);
		robot_data.com(0) = *com_in[0];		// TODO: This initialization did not work
		robot_data.com(1) = *com_in[1];
		robot_data.com(2) = *com_in[2];
		robot_data.left_foot.position[0] 	= *left_ankle_in[0] - left_foot.ankle_pos_local(0);
		robot_data.left_foot.position[1] 	= *left_ankle_in[1];
		robot_data.left_foot.position[2] 	= *left_ankle_in[2];
		robot_data.right_foot.position[0]  	= *right_ankle_in[0] - right_foot.ankle_pos_local(0);
		robot_data.right_foot.position[1]  	= *right_ankle_in[1];
		robot_data.right_foot.position[2]  	= *right_ankle_in[2];
		robot_data.max_foot_vel = 1.;
		robot_data.security_margin = kSecurityMargin;

		// Feasibility hulls:
		// ------------------
		const int num_vert_foot_pos = 5;
		double DefaultFPosEdgesX[num_vert_foot_pos] = {-0.3, -0.2, 0.0, 0.2, 0.3};
		double DefaultFPosEdgesY[num_vert_foot_pos] = {-0.2, -0.3, -0.4, -0.3, -0.2};

		robot_data.left_foot_pos_hull.Resize(num_vert_foot_pos);
		robot_data.right_foot_pos_hull.Resize(num_vert_foot_pos);
		for (int i = 0; i < num_vert_foot_pos; ++i) {
			robot_data.left_foot_pos_hull.x_vec(i)  = DefaultFPosEdgesX[i];
			robot_data.left_foot_pos_hull.y_vec(i)  = DefaultFPosEdgesY[i];
			robot_data.right_foot_pos_hull.x_vec(i) = DefaultFPosEdgesX[i];
			robot_data.right_foot_pos_hull.y_vec(i) = -DefaultFPosEdgesY[i];
		}

		double feet_distance_y = *left_ankle_in[1] - *right_ankle_in[1];
		robot_data.SetCoPHulls(feet_distance_y);

		robot_data.max_foot_height = kMaxFootHeight;

		walk->SetVelReference(0.0, 0.0, 0.0);
		walk->SetCPReference(*com_in[0], *com_in[1], *com_in[2], *com_in[3]);
		walk->Init(robot_data);
		RigidBodySystem *robot = walk->robot();
		robot->com()->state().x[0] = *com_in[0];
		robot->com()->state().y[0] = *com_in[1];
		robot->com()->state().z[0] = *com_in[2];
		robot->com()->state().x[1] = *com_in[3];
		robot->com()->state().y[1] = *com_in[4];
		robot->com()->state().z[1] = 0.;
		robot->com()->state().x[2] = kGravity / *com_in[2] * (*com_in[0] - *cop_in[0]);
		robot->com()->state().y[2] = kGravity / *com_in[2] * (*com_in[1] - *cop_in[1]);
		robot->com()->state().z[2] = 0.;

		ssSetIWorkValue(S, 0, 1);//Is initialized

	} // End of initialization

	// INPUT:
	// ------
	walk->SetVelReference(*vel_ref[0], *vel_ref[1], *vel_ref[2]);
	walk->SetPosReference(*pos_ref[0], *pos_ref[1]);
	walk->SetCPReference(*cp_ref[0], *cp_ref[1], *cp_ref[2], *cp_ref[3]);
	RigidBodySystem *robot = walk->robot();
	if (is_closed_loop_in > 0.5) {
		robot->com()->state().x[0] = *com_in[0];
		robot->com()->state().y[0] = *com_in[1];
		robot->com()->state().x[1] = *com_in[3];
		robot->com()->state().y[1] = *com_in[4];
		robot->com()->state().x[2] = kGravity / *com_in[2] * (*com_in[0] - *cop_in[0]);
		robot->com()->state().y[2] = kGravity / *com_in[2] * (*com_in[1] - *cop_in[1]);

		robot->left_foot()->force_sensor().force_z = *lfoot_wrench_in[2];
		robot->right_foot()->force_sensor().force_z = *rfoot_wrench_in[2];
	}

	// Run simulation:
	// ---------------
	double curr_time = ssGetT(S);
	walk->clock().ResetLocal();
	int time_online = walk->clock().StartCounter();
	const MPCSolution &solution = walk->Go(curr_time);
	walk->clock().StopCounter(time_online);

	// Assign to the output:
	// ---------------------
	com[0]      = walk->output().com.x;
	com[1]      = walk->output().com.y;
	com[2]      = walk->output().com.z;
	dcom[0]     = walk->output().com.dx;
	dcom[1]     = walk->output().com.dy;
	dcom[2]     = walk->output().com.dz;
	ddcom[0]    = walk->output().com.ddx;
	ddcom[1]    = walk->output().com.ddy;
	ddcom[2]    = walk->output().com.ddz;  //TODO:

	cop[0] = walk->output().cop.x;
	cop[1] = walk->output().cop.y;

	// Left foot:
	double ankle_pos_loc_x = -0.035;
	p_left[0]   = walk->output().left_foot.x + ankle_pos_loc_x;
	p_left[1]   = walk->output().left_foot.y;
	p_left[2]   = walk->output().left_foot.z;
	p_left[3]   = walk->output().left_foot.yaw;
	dp_left[0]  = walk->output().left_foot.dx;
	dp_left[1]  = walk->output().left_foot.dy;
	dp_left[2]  = walk->output().left_foot.dz;
	dp_left[3]  = walk->output().left_foot.dyaw;
	ddp_left[0] = walk->output().left_foot.ddx;
	ddp_left[1] = walk->output().left_foot.ddy;
	ddp_left[2] = walk->output().left_foot.ddz;
	ddp_left[3] = walk->output().left_foot.ddyaw;
	// Right foot:
	p_right[0]      = walk->output().right_foot.x + ankle_pos_loc_x;
	p_right[1]      = walk->output().right_foot.y;
	p_right[2]      = walk->output().right_foot.z;
	p_right[3]      = walk->output().right_foot.yaw;
	dp_right[0]     = walk->output().right_foot.dx;
	dp_right[1]     = walk->output().right_foot.dy;
	dp_right[2]     = walk->output().right_foot.dz;
	dp_right[3]     = walk->output().right_foot.dyaw;
	ddp_right[0]    = walk->output().right_foot.ddx;
	ddp_right[1]    = walk->output().right_foot.ddy;
	ddp_right[2]    = walk->output().right_foot.ddz;
	ddp_right[3]    = walk->output().right_foot.ddyaw;

	// Previewed motions:
	// ------------------
	if (kDebug == 1) {
		int num_samples = walk->mpc_parameters().num_samples_horizon_max - 1;
		for (int sample = 0; sample < num_samples; ++sample) {
			// CoM:
			com_prw[sample]                     = solution.sampling_times_vec[sample+1];
			com_prw[num_samples + sample]       = solution.com_prw.pos.x_vec[sample];
			com_prw[2 * num_samples + sample]   = solution.com_prw.pos.y_vec[sample];
			com_prw[3 * num_samples + sample]   = walk->output().com.z;
			// CoP:
			cop_prw[sample]                     = solution.sampling_times_vec[sample+1];
			cop_prw[num_samples + sample]       = solution.com_prw.cop.x_vec[sample];
			cop_prw[2 * num_samples + sample]   = solution.com_prw.cop.y_vec[sample];
			// CoM control vector:
			com_control_prw[sample]                     = solution.sampling_times_vec[sample];
			com_control_prw[num_samples + sample]       = solution.com_prw.control.x_vec[sample];
			com_control_prw[2 * num_samples + sample]   = solution.com_prw.control.y_vec[sample];
			// CP:
			cp_prw[sample]                      = solution.sampling_times_vec[sample+1];
			cp_prw[num_samples + sample]        = solution.com_prw.cp.x_vec[sample];
			cp_prw[2 * num_samples + sample]    = solution.com_prw.cp.y_vec[sample];
		}

		int num_steps_prw = solution.support_states_vec.back().step_number;
		int num_var_ff_x = 2 * num_samples;
		int num_var_ff_y = 2 * num_samples + num_steps_prw;
		if (walk->mpc_parameters().formulation == DECOUPLED_MODES) {
			num_var_ff_x += 2;
			num_var_ff_y += 2;
		}
		if (num_steps_prw > 0) {
			first_foot_prw[0] = solution.qp_solution_vec[num_var_ff_x];
			first_foot_prw[1] = solution.qp_solution_vec[num_var_ff_y];
		} else {
			first_foot_prw[0] = solution.support_states_vec.front().x;
			first_foot_prw[1] = solution.support_states_vec.front().y;
		}
		// Change of the current support state (time instant)
		const SupportState &current_support = solution.support_states_vec.front();
		const SupportState &next_support = solution.support_states_vec[1];
		if (current_support.state_changed) {
			support[0] = current_support.start_time;
		} else if (current_support.transitional_ds && !next_support.transitional_ds) {
			support[0] = current_support.start_time + walk->mpc_parameters().period_trans_ds();  // TODO: 0.1 is temporary solution
		} else {
			support[0] = 0.0;
		}
		support[1] = current_support.phase;
		support[2] = current_support.foot;

		// Analysis:
		// ---------
		analysis[0] = solution.analysis.resolution_time;
		analysis[1] = solution.analysis.num_iterations;
		analysis[2] = solution.analysis.objective_value;
		int num_counters = walk->clock().GetNumCounters();
		for (int counter = num_counters - 1; counter >= 0; counter--) {
			analysis[counter + 3] = walk->clock().GetTime(counter);
		}

		cur_state[0] = robot->com()->state().x[0];
		cur_state[1] = robot->com()->state().x[1];
		cur_state[2] = robot->com()->state().x[2];

		cur_state[3] = robot->com()->state().y[0];
		cur_state[4] = robot->com()->state().y[1];
		cur_state[5] = robot->com()->state().y[2];

		cur_state[6] = robot->com()->state().z[0];
		cur_state[7] = robot->com()->state().z[1];
		cur_state[8] = robot->com()->state().z[2];

		sim_parameters[0] =	walk->mpc_parameters().num_samples_horizon_max - 1;
		sim_parameters[1] = walk->mpc_parameters().num_samples_step;
		sim_parameters[2] = walk->mpc_parameters().num_samples_dsss;
		sim_parameters[3] = walk->mpc_parameters().num_steps_ssds;
		sim_parameters[4] = walk->mpc_parameters().period_qpsample;
		sim_parameters[5] = walk->mpc_parameters().period_recomputation;
		sim_parameters[6] = walk->mpc_parameters().period_actsample;
	}
}

static void mdlTerminate(SimStruct *S) {
	Walkgen *walk = static_cast<Walkgen *>(ssGetPWork(S)[0]);
	Debug::Cout("Distribution of ticks", walk->clock().ticks_distr_vec());
	delete walk;
}

// Required S-function trailer:
// ----------------------------
#ifdef  MATLAB_MEX_FILE    // Is this file being compiled as a MEX-file?
#include "simulink.c"      // MEX-file interface mechanism
#else
#include "cg_sfun.h"       // Code generation registration function
#endif

#ifdef __cplusplus
}
#endif
