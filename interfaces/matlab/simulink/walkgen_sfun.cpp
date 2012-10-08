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
	ssSetNumSFcnParams(S, 11);

	// Parameter mismatch?
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
		return;
	}

	// Specify I/O
	if (!ssSetNumInputPorts(S, 10)) return;
	ssSetInputPortWidth(S, 0, 3);     //vel_ref

	ssSetInputPortWidth(S, 1, 6);     //com_in
	ssSetInputPortWidth(S, 2, 3);     //left_ankle_in
	ssSetInputPortWidth(S, 3, 3);     //right_ankle_in
	ssSetInputPortWidth(S, 4, 1);     //left_yaw
	ssSetInputPortWidth(S, 5, 1);     //right_yaw
	ssSetInputPortWidth(S, 6, 4);     //foot_geometry
	ssSetInputPortWidth(S, 7, 1);     //closed_loop
	ssSetInputPortWidth(S, 8, 2);     //cop
	ssSetInputPortWidth(S, 9, 1);     //reset_in

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

	if (!ssSetNumOutputPorts(S,16)) return;
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
	const int kNumSamplesHorizon      = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 0)));
	const int kNumSamplesStep         = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 1)));
	const int num_steps_max           = kNumSamplesHorizon / kNumSamplesStep + 1;
	ssSetOutputPortWidth(S, 10, num_steps_max);               //first_foot_prw
	ssSetOutputPortWidth(S, 11, 4 * kNumSamplesHorizon);     //com_prw (sample_instants, x, y, z)
	ssSetOutputPortWidth(S, 12, 3 * kNumSamplesHorizon);     //cop_prw (sample_instants, x, y)
	ssSetOutputPortWidth(S, 13, 3);       //support
	ssSetOutputPortWidth(S, 14, 30);      //analysis
	ssSetOutputPortWidth(S, 15, 3 * kNumSamplesHorizon);	//com_control_prw

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
	const static int kDebug = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 10)));

	MPCParameters mpc_parameters;
	mpc_parameters.num_samples_horizon  = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 0)));
	mpc_parameters.num_samples_step     = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 1)));
	mpc_parameters.num_samples_dsss     = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 2)));
	mpc_parameters.num_steps_ssds       = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 3)));
	mpc_parameters.period_qpsample      = *mxGetPr(ssGetSFcnParam(S, 4));
	mpc_parameters.period_mpcsample     = *mxGetPr(ssGetSFcnParam(S, 5));
	mpc_parameters.period_actsample     = *mxGetPr(ssGetSFcnParam(S, 6));
	mpc_parameters.solver.num_wsrec     = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 8)));
	mpc_parameters.warmstart					= false;
	if (kDebug == 0) {
		mpc_parameters.interpolate_whole_horizon	= false;
		mpc_parameters.solver.analysis			    = false;
	} else {
		mpc_parameters.interpolate_whole_horizon	= true;
		mpc_parameters.solver.analysis			    = true;
	}
	mpc_parameters.solver.name                  = QPOASES;

	mpc_parameters.dynamics_order               = static_cast<DynamicsOrder>(static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 9))));

	Walkgen *walk = new Walkgen;
	walk->Init(mpc_parameters);

	ssSetPWorkValue(S, 0, (void*)walk);
	ssSetIWorkValue(S, 0, 0);     //MPCWalkgen not initialized

}

static void mdlOutputs(SimStruct *S, int_T tid) {
	const double kSecurityMargin = *mxGetPr(ssGetSFcnParam(S, 7));
	const double kMaxFootHeight = 0.03;
	const double kGravity = 9.81;
	const static int kDebug = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 10)));


	InputRealPtrsType vel_ref         = ssGetInputPortRealSignalPtrs(S, 0);
	InputRealPtrsType com_in          = ssGetInputPortRealSignalPtrs(S, 1);
	InputRealPtrsType left_ankle_in   = ssGetInputPortRealSignalPtrs(S, 2);
	InputRealPtrsType right_ankle_in  = ssGetInputPortRealSignalPtrs(S, 3);
	InputRealPtrsType left_yaw        = ssGetInputPortRealSignalPtrs(S, 4);
	InputRealPtrsType right_yaw       = ssGetInputPortRealSignalPtrs(S, 5);
	InputRealPtrsType foot_geometry   = ssGetInputPortRealSignalPtrs(S, 6);
	InputRealPtrsType closed_loop_in  = ssGetInputPortRealSignalPtrs(S, 7);
	InputRealPtrsType cop_in          = ssGetInputPortRealSignalPtrs(S, 8);
	InputRealPtrsType reset_in        = ssGetInputPortRealSignalPtrs(S, 9);

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

	Walkgen *walk = (Walkgen *)ssGetPWorkValue(S, 0);

	// Begin initialization of the robot:
	// ----------------------------------
	if (ssGetIWorkValue(S, 0) == 0) {
		FootData leftFoot;
		leftFoot.anklePositionInLocalFrame << 0, 0, 0;      //0, 0, 0.105;//TODO:Is not used
		leftFoot.soleHeight = *foot_geometry[2] - *foot_geometry[3];
		leftFoot.soleWidth = *foot_geometry[0] - *foot_geometry[1];
		leftFoot.SetEdges(*foot_geometry[0], *foot_geometry[1], *foot_geometry[2], *foot_geometry[3], kSecurityMargin);
		FootData rightFoot;
		rightFoot.anklePositionInLocalFrame << 0, 0, 0;     //0, 0, 0.105;//TODO:Is not used
		rightFoot.soleHeight = *foot_geometry[2] - *foot_geometry[3];
		rightFoot.soleWidth = *foot_geometry[0] - *foot_geometry[1];
		rightFoot.SetEdges(*foot_geometry[0], *foot_geometry[1], *foot_geometry[2], *foot_geometry[3], kSecurityMargin);

		HipYawData leftHipYaw;
		leftHipYaw.lowerBound                   = -0.523599;
		leftHipYaw.upperBound                   = 0.785398;
		leftHipYaw.lowerVelocityBound           = -3.54108;
		leftHipYaw.upperVelocityBound           = 3.54108;
		leftHipYaw.lowerAccelerationBound       = -0.1;
		leftHipYaw.upperAccelerationBound       = 0.1;
		HipYawData rightHipYaw                  = leftHipYaw;

		RobotData robot_data(leftFoot, rightFoot, leftHipYaw, rightHipYaw, 0.0);

		robot_data.com(0) = *com_in[0];		// TODO: This initialization did not work
		robot_data.com(1) = *com_in[1];
		robot_data.com(2) = *com_in[2];
		robot_data.leftFoot.position[0] = *left_ankle_in[0];
		robot_data.leftFoot.position[1] = *left_ankle_in[1];
		robot_data.leftFoot.position[2] = *left_ankle_in[2];
		robot_data.right_foot.position[0]  = *right_ankle_in[0];
		robot_data.right_foot.position[1]  = *right_ankle_in[1];
		robot_data.right_foot.position[2]  = *right_ankle_in[2];
		robot_data.max_foot_vel = 1.;
		robot_data.security_margin = kSecurityMargin;

		// Feasibility hulls:
		// ------------------
		const int nbVertFeet = 5;
		// Feasible foot positions
		double DefaultFPosEdgesX[nbVertFeet] = {-0.2, -0.2, 0.0, 0.2, 0.2};
		double DefaultFPosEdgesY[nbVertFeet] = {-0.2, -0.3, -0.4, -0.3, -0.2};

		robot_data.left_foot_pos_hull.Resize(nbVertFeet);
		robot_data.right_foot_pos_hull.Resize(nbVertFeet);
		for (int i = 0; i < nbVertFeet; ++i) {
			robot_data.left_foot_pos_hull.x_vec(i)  = DefaultFPosEdgesX[i];
			robot_data.left_foot_pos_hull.y_vec(i)  = DefaultFPosEdgesY[i];
			robot_data.right_foot_pos_hull.x_vec(i) = DefaultFPosEdgesX[i];
			robot_data.right_foot_pos_hull.y_vec(i) = -DefaultFPosEdgesY[i];
		}

		double feet_distance_y = *left_ankle_in[1] - *right_ankle_in[1];
		robot_data.SetCoPHulls(feet_distance_y);

		robot_data.max_foot_height = kMaxFootHeight;

		walk->SetReference(0.0, 0.0, 0.0);
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
	walk->SetReference(*vel_ref[0], *vel_ref[1], *vel_ref[2]);
	RigidBodySystem *robot = walk->robot();
	if (*closed_loop_in[0] > 0.5) {// TODO: Is there a better way for switching?
		robot->com()->state().x[0] = *com_in[0];
		robot->com()->state().y[0] = *com_in[1];
		robot->com()->state().x[1] = *com_in[3];
		robot->com()->state().y[1] = *com_in[4];
		robot->com()->state().x[2] = kGravity / *com_in[2] * (*com_in[0] - *cop_in[0]);
		robot->com()->state().y[2] = kGravity / *com_in[2] * (*com_in[1] - *cop_in[1]);
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
	com[0] = walk->output().com.x;
	com[1] = walk->output().com.y;
	com[2] = walk->output().com.z;
	dcom[0] = walk->output().com.dx;
	dcom[1] = walk->output().com.dy;
	dcom[2] = walk->output().com.dz;
	ddcom[0] = walk->output().com.ddx;
	ddcom[1] = walk->output().com.ddy;
	ddcom[2] = walk->output().com.ddz;  //TODO: 

	cop[0] = walk->output().cop.x;
	cop[1] = walk->output().cop.y;

	// Left foot:
	p_left[0] = walk->output().left_foot.x;
	p_left[1] = walk->output().left_foot.y;
	p_left[2] = walk->output().left_foot.z;
	p_left[3] = walk->output().left_foot.yaw;
	dp_left[0] = walk->output().left_foot.dx;
	dp_left[1] = walk->output().left_foot.dy;
	dp_left[2] = walk->output().left_foot.dz;
	dp_left[3] = walk->output().left_foot.dyaw;
	ddp_left[0] = walk->output().left_foot.ddx;
	ddp_left[1] = walk->output().left_foot.ddy;
	ddp_left[2] = walk->output().left_foot.ddz;
	ddp_left[3] = walk->output().left_foot.ddyaw;
	// Right foot:
	p_right[0] = walk->output().right_foot.x;
	p_right[1] = walk->output().right_foot.y;
	p_right[2] = walk->output().right_foot.z;
	p_right[3] = walk->output().right_foot.yaw;
	dp_right[0] = walk->output().right_foot.dx;
	dp_right[1] = walk->output().right_foot.dy;
	dp_right[2] = walk->output().right_foot.dz;
	dp_right[3] = walk->output().right_foot.dyaw;
	ddp_right[0] = walk->output().right_foot.ddx;
	ddp_right[1] = walk->output().right_foot.ddy;
	ddp_right[2] = walk->output().right_foot.ddz;
	ddp_right[3] = walk->output().right_foot.ddyaw;

	// Previewed motions:
	// ------------------
	if (kDebug == 1) {
	int nbsamples = solution.support_states_vec.size() - 1;
	for (int sample = 0; sample < nbsamples; ++sample) {
		// CoM:
		com_prw[sample] = solution.sampling_times_vec[sample+1];
		com_prw[nbsamples + sample] = solution.com_prw.pos.x_vec[sample];
		com_prw[2 * nbsamples + sample] = solution.com_prw.pos.y_vec[sample];
		com_prw[3 * nbsamples + sample] = walk->output().com.z;
		// CoP:
		cop_prw[sample] = solution.sampling_times_vec[sample+1];
		cop_prw[nbsamples + sample] = solution.cop_prw.pos.x_vec[sample];
		cop_prw[2 * nbsamples + sample] = solution.cop_prw.pos.y_vec[sample];
		// CoM control vector:
		com_control_prw[sample] = solution.sampling_times_vec[sample+1];
		com_control_prw[nbsamples + sample] = solution.com_prw.control.x_vec[sample];
		com_control_prw[2 * nbsamples + sample] = solution.com_prw.control.y_vec[sample];
	}

	int nbsteps_prw = solution.support_states_vec.back().step_number;
	if (nbsteps_prw > 0) {
		first_foot_prw[0] = solution.qp_solution_vec[2 * nbsamples];
		first_foot_prw[1] = solution.qp_solution_vec[2 * nbsamples + nbsteps_prw];
	} else {
		first_foot_prw[0] = solution.support_states_vec.front().x;
		first_foot_prw[1] = solution.support_states_vec.front().y;
	}
	// Change of the current support state (time instant)
	const SupportState &current_support = solution.support_states_vec.front();
	const SupportState &next_support = solution.support_states_vec[1];
	if (current_support.state_changed) {
		support[0] = current_support.start_time;
	} else if (next_support.transitional_ds) {
		support[0] = current_support.time_limit - 0.1;  // TODO: 0.1 is temporary solution
	} else {
		support[0] = 0.0;
	}
	support[1] = current_support.phase;
	support[2] = current_support.foot;


	// Analysis:
	// ---------
	analysis[0] = solution.analysis.resolution_time;
	analysis[1] = solution.analysis.num_iterations;
	int num_counters = walk->clock().GetNumCounters();
	//for (int i = num_counters - 1; i >= 0; i--) {
	analysis[/*i + */2] = walk->clock().GetTime(time_online);
	//}
	}

}

static void mdlTerminate(SimStruct *S) {
	Walkgen *walk = static_cast<Walkgen *>(ssGetPWork(S)[0]);
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
