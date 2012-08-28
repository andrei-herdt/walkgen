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

static void mdlInitializeSizes(SimStruct *S)
{
  // No expected parameters
  ssSetNumSFcnParams(S, 7);

  // Parameter mismatch will be reported by Simulink
  if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
    return;
  }

  // Specify I/O
  if (!ssSetNumInputPorts(S, 9)) return;
  ssSetInputPortWidth(S, 0, 3);     //vel_ref

  ssSetInputPortWidth(S, 1, 6);     //com_in
  ssSetInputPortWidth(S, 2, 3);     //left_ankle_in
  ssSetInputPortWidth(S, 3, 3);     //right_ankle_in
  ssSetInputPortWidth(S, 4, 1);     //left_yaw
  ssSetInputPortWidth(S, 5, 1);     //right_yaw
  ssSetInputPortWidth(S, 6, 4);     //foot_geometry
  ssSetInputPortWidth(S, 7, 1);     //closed_loop
  ssSetInputPortWidth(S, 8, 2);     //cop

  ssSetInputPortDirectFeedThrough(S, 0, 1);
  ssSetInputPortDirectFeedThrough(S, 1, 1);
  ssSetInputPortDirectFeedThrough(S, 2, 1);
  ssSetInputPortDirectFeedThrough(S, 3, 1);
  ssSetInputPortDirectFeedThrough(S, 4, 1);
  ssSetInputPortDirectFeedThrough(S, 5, 1);
  ssSetInputPortDirectFeedThrough(S, 6, 1);
  ssSetInputPortDirectFeedThrough(S, 7, 1);
  ssSetInputPortDirectFeedThrough(S, 8, 1);

  if (!ssSetNumOutputPorts(S,15)) return;
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
  const int num_samples_horizon      = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 0)));
  const int num_samples_step         = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 1)));
  const int num_steps_max = num_samples_horizon / num_samples_step + 1;
  ssSetOutputPortWidth(S, 10, num_steps_max);               //first_foot_prw
  ssSetOutputPortWidth(S, 11, 4 * num_samples_horizon);     //com_prw (sample_instants, x, y, z)
  ssSetOutputPortWidth(S, 12, 3 * num_samples_horizon);     //cop_prw (sample_instants, x, y)

  ssSetOutputPortWidth(S, 13, 1);       //support
  ssSetOutputPortWidth(S, 14, 30);      //analysis

  ssSetNumSampleTimes(S, 1);

  ssSetNumPWork(S, 1);
  ssSetNumIWork(S, 1);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
  ssSetSampleTime(S, 0, 0.001);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
  WalkgenAbstract *walk = createWalkgen();
  ssSetPWorkValue(S, 0, (void*)walk);
  ssSetIWorkValue(S, 0, 0);     //MPCWalkgen not initialized

}

static void mdlOutputs(SimStruct *S, int_T tid) {
  InputRealPtrsType vel_ref         = ssGetInputPortRealSignalPtrs(S, 0);
  InputRealPtrsType com_in          = ssGetInputPortRealSignalPtrs(S, 1);
  InputRealPtrsType left_ankle_in   = ssGetInputPortRealSignalPtrs(S, 2);
  InputRealPtrsType right_ankle_in  = ssGetInputPortRealSignalPtrs(S, 3);
  InputRealPtrsType left_yaw        = ssGetInputPortRealSignalPtrs(S, 4);
  InputRealPtrsType right_yaw       = ssGetInputPortRealSignalPtrs(S, 5);
  InputRealPtrsType foot_geometry   = ssGetInputPortRealSignalPtrs(S, 6);
  InputRealPtrsType closed_loop_in  = ssGetInputPortRealSignalPtrs(S, 7);
  InputRealPtrsType cop_in          = ssGetInputPortRealSignalPtrs(S, 8);

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

  
  const int num_samples_horizon      = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 0)));
  const int num_samples_step         = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 1)));
  const int num_samples_dsss         = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 2)));
  const int num_steps_ssds           = static_cast<int>(*mxGetPr(ssGetSFcnParam(S, 3)));
  const double sample_period_qp      = *mxGetPr(ssGetSFcnParam(S, 4));
  const double sample_period_first   = *mxGetPr(ssGetSFcnParam(S, 5));
  const double sample_period_act     = *mxGetPr(ssGetSFcnParam(S, 6));
    
  WalkgenAbstract *walk = (WalkgenAbstract *)ssGetPWorkValue(S, 0);

  // Initialization:
  // ---------------
  if (ssGetIWorkValue(S, 0) == 0) {// Not initialized

    walk->reference(*vel_ref[0], *vel_ref[1], *vel_ref[2]);

    FootData leftFoot;      // TODO: Never used
    double security_margin = 0.0;
    leftFoot.anklePositionInLocalFrame << 0, 0, 0;      //0, 0, 0.105;//TODO:Is not used
    leftFoot.soleHeight = *foot_geometry[2] - *foot_geometry[3] - security_margin;
    leftFoot.soleWidth = *foot_geometry[0] - *foot_geometry[1] - security_margin;
    FootData rightFoot;
    rightFoot.anklePositionInLocalFrame << 0, 0, 0;     //0, 0, 0.105;//TODO:Is not used
    rightFoot.soleHeight = *foot_geometry[2] - *foot_geometry[3] - security_margin;
    rightFoot.soleWidth = *foot_geometry[0] - *foot_geometry[1] - security_margin;

    HipYawData leftHipYaw;
    leftHipYaw.lowerBound = -0.523599;
    leftHipYaw.upperBound = 0.785398;
    leftHipYaw.lowerVelocityBound = -3.54108;
    leftHipYaw.upperVelocityBound = 3.54108;
    leftHipYaw.lowerAccelerationBound = -0.1;
    leftHipYaw.upperAccelerationBound = 0.1;
    HipYawData rightHipYaw = leftHipYaw;

    MPCData mpc_data;
    mpc_data.nbsamples_qp       = num_samples_horizon;
    mpc_data.nbqpsamples_step   = num_samples_step;
    mpc_data.nbqpsamples_dsss   = num_samples_dsss;
    mpc_data.nbsteps_ssds       = num_steps_ssds;
    mpc_data.period_qpsample    = sample_period_qp;
    mpc_data.period_mpcsample   = sample_period_first;
    mpc_data.period_actsample   = sample_period_act;
    mpc_data.ponderation.JerkMin[0] = 0.00001;
    mpc_data.ponderation.JerkMin[1] = 0.00001;
    mpc_data.warmstart = true;
    mpc_data.interpolate_preview = true;
    mpc_data.solver.analysis = true;
    mpc_data.solver.name = QPOASES;
    mpc_data.solver.num_wsrec = 2;

    RobotData robot_data(leftFoot, rightFoot, leftHipYaw, rightHipYaw, 0.0);

    robot_data.com(0) = *com_in[0];
    robot_data.com(1) = *com_in[1];
    robot_data.com(2) = *com_in[2];

    robot_data.leftFootPos(0) = *left_ankle_in[0];
    robot_data.leftFootPos(1) = *left_ankle_in[1];
    robot_data.leftFootPos(2) = *left_ankle_in[2];

    robot_data.rightFootPos(0) = *right_ankle_in[0];
    robot_data.rightFootPos(1) = *right_ankle_in[1];
    robot_data.rightFootPos(2) = *right_ankle_in[2];

    // Feasible hulls:
    // ---------------
    const int nbVertFeet = 5;
    // Feasible foot positions
    double DefaultFPosEdgesX[nbVertFeet] = {-0.28, -0.2, 0.0, 0.2, 0.28};
    double DefaultFPosEdgesY[nbVertFeet] = {-0.2, -0.3, -0.4, -0.3, -0.2};

    robot_data.leftFootHull.resize(nbVertFeet);
    robot_data.rightFootHull.resize(nbVertFeet);
    for (int i=0; i < nbVertFeet; ++i) {
      robot_data.leftFootHull.x(i)=DefaultFPosEdgesX[i];
      robot_data.leftFootHull.y(i)=DefaultFPosEdgesY[i];
      robot_data.rightFootHull.x(i)=DefaultFPosEdgesX[i];
      robot_data.rightFootHull.y(i)=-DefaultFPosEdgesY[i];
    }

    // Constraints on the CoP
    // TODO: Should not be hardcoded
    const int nbVertCoP = 4;
    double DefaultCoPSSEdgesX[nbVertCoP] = {0.0686, 0.0686, -0.0686, -0.0686};
    double DefaultCoPSSEdgesY[nbVertCoP] = {0.029, -0.029, -0.029, 0.029};
    double DefaultCoPDSEdgesX[nbVertCoP] = {0.0686, 0.0686, -0.0686, -0.0686};
    double DefaultCoPDSEdgesY[nbVertCoP] = {0.029, -0.229, -0.229, 0.029};

    robot_data.CoPLeftSSHull.resize(nbVertCoP);
    robot_data.CoPRightSSHull.resize(nbVertCoP);
    robot_data.CoPLeftDSHull.resize(nbVertCoP);
    robot_data.CoPRightDSHull.resize(nbVertCoP);
    for (int i = 0; i < nbVertCoP; ++i) {
      robot_data.CoPLeftSSHull.x(i) = DefaultCoPSSEdgesX[i];
      robot_data.CoPLeftSSHull.y(i) = DefaultCoPSSEdgesY[i];
      robot_data.CoPLeftDSHull.x(i) = DefaultCoPDSEdgesX[i];
      robot_data.CoPLeftDSHull.y(i) = DefaultCoPDSEdgesY[i];

      robot_data.CoPRightSSHull.x(i) = DefaultCoPSSEdgesX[i];
      robot_data.CoPRightSSHull.y(i) =- DefaultCoPSSEdgesY[i];
      robot_data.CoPRightDSHull.x(i) = DefaultCoPDSEdgesX[i];
      robot_data.CoPRightDSHull.y(i) =- DefaultCoPDSEdgesY[i];
    }
 
    walk->Init(robot_data, mpc_data);
    ssSetIWorkValue(S, 0, 1);//Is initialized
  }

  // INPUT:
  // ------
  walk->reference(*vel_ref[0], *vel_ref[1], *vel_ref[2]);
  double kGravity = 9.81;
  RigidBodySystem *robot = walk->robot();
  if (*closed_loop_in[0] > 0.5) {// TODO: Is there a better way for switching
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
  MPCSolution solution;
  int time_online = walk->watch()->StartCounting();
  solution = walk->online(curr_time);
  walk->watch()->StopCounting(time_online);
  
  // Assign to the output:
  // ---------------------
  com[0] = walk->output().com.x;
  com[1] = walk->output().com.y;
  com[2] = walk->output().com.z;
  dcom[0] = walk->output().com.dx;
  dcom[1] = walk->output().com.dy;
  dcom[2] = walk->bodyState(COM).z(1);
  ddcom[0] = walk->output().com.ddx;
  ddcom[1] = walk->output().com.ddy;
  ddcom[2] = walk->bodyState(COM).z(2);

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
  int nbsamples = solution.support_states_vec.size() - 1;
  for (int sample = 0; sample < nbsamples; ++sample) {
    // CoM:
    com_prw[sample] = solution.samplingTimes_vec[sample+1];
    com_prw[nbsamples + sample] = solution.com_prw.pos.x_vec[sample];
    com_prw[2 * nbsamples + sample] = solution.com_prw.pos.y_vec[sample];
    com_prw[3 * nbsamples + sample] = walk->bodyState(COM).z(0);
    // CoP:    
    cop_prw[sample] = solution.samplingTimes_vec[sample+1];
    cop_prw[nbsamples + sample] = solution.cop_prw.pos.x_vec[sample];
    cop_prw[2 * nbsamples + sample] = solution.cop_prw.pos.y_vec[sample];
  }

  int nbsteps_prw = solution.support_states_vec.back().stepNumber;
  if (nbsteps_prw > 0) {
    first_foot_prw[0] = solution.qpSolution[2 * nbsamples];
    first_foot_prw[1] = solution.qpSolution[2 * nbsamples + nbsteps_prw];
  } else {
    first_foot_prw[0] = solution.support_states_vec.front().x;
    first_foot_prw[1] = solution.support_states_vec.front().y;
  }
  // Change of the current support state (time instant)
  SupportState &current_support = solution.support_states_vec.front();
  SupportState &next_support = solution.support_states_vec[1];
  if (current_support.state_changed) {
    support[0] = current_support.start_time;
  } else if (next_support.transitional_ds) {
    support[0] = current_support.time_limit - 0.1;  // TODO: 0.1 is temporary solution
  } else {
    support[0] = 0.0;
  }

  // Analysis:
  // ---------
  analysis[0] = solution.analysis.resolution_time;
  analysis[1] = solution.analysis.num_iterations;
  int num_counters = walk->watch()->GetNumCounters();
  for (int i = num_counters - 1; i >= 0; i--) {
	  analysis[i + 2] = walk->watch()->GetLastMeasure();
  }

}

static void mdlTerminate(SimStruct *S)
{
  WalkgenAbstract *walk = static_cast<WalkgenAbstract *>(ssGetPWork(S)[0]);
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
