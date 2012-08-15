/* Copyright 2003-2004 The MathWorks, Inc. */
#include <iostream>
#include <stdio.h>
using namespace std;
// *******************************************************************
// **** To build this mex function use: mex sfun_cppcount_cpp.cpp ****
// *******************************************************************

#include <mpc-walkgen/rigid-body-system.h>// TODO: This does not belong to the API
#include <walkgen-abstract.h>

using namespace Eigen;
using namespace MPCWalkgen;


#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  walkgen_cpp

// Need to include simstruc.h for the definition of the SimStruct and
// its associated macro definitions.
#include "simstruc.h"

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
  !mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

// Function: mdlInitializeSizes ===============================================
// Abstract:
//    The sizes information is used by Simulink to determine the S-function
//    block's characteristics (number of inputs, outputs, states, etc.).
static void mdlInitializeSizes(SimStruct *S)
{
  // No expected parameters
  ssSetNumSFcnParams(S, 0);

  // Parameter mismatch will be reported by Simulink
  if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
    return;
  }

  // Specify I/O
  if (!ssSetNumInputPorts(S, 9)) return;
  ssSetInputPortWidth(S, 0, 3);//vel_ref

  ssSetInputPortWidth(S, 1, 6);//com_in
  ssSetInputPortWidth(S, 2, 3);//left_ankle_in
  ssSetInputPortWidth(S, 3, 3);//right_ankle_in
  ssSetInputPortWidth(S, 4, 1);//left_yaw
  ssSetInputPortWidth(S, 5, 1);//right_yaw
  ssSetInputPortWidth(S, 6, 4);//foot_geometry
  ssSetInputPortWidth(S, 7, 1);//closed_loop
  ssSetInputPortWidth(S, 8, 2);//cop

  //ssSetInputPortWidth(S, 0, DYNAMICALLY_SIZED);
  ssSetInputPortDirectFeedThrough(S, 0, 1);
  ssSetInputPortDirectFeedThrough(S, 1, 1);
  ssSetInputPortDirectFeedThrough(S, 2, 1);
  ssSetInputPortDirectFeedThrough(S, 3, 1);
  ssSetInputPortDirectFeedThrough(S, 4, 1);
  ssSetInputPortDirectFeedThrough(S, 5, 1);
  ssSetInputPortDirectFeedThrough(S, 6, 1);
  ssSetInputPortDirectFeedThrough(S, 7, 1);
  ssSetInputPortDirectFeedThrough(S, 8, 1);

  if (!ssSetNumOutputPorts(S,14)) return;
  // Realized motions
  ssSetOutputPortWidth(S, 0, 3);//com
  ssSetOutputPortWidth(S, 1, 3);//dcom
  ssSetOutputPortWidth(S, 2, 3);//ddcom
  ssSetOutputPortWidth(S, 3, 2);//cop
  ssSetOutputPortWidth(S, 4, 4);//p_left
  ssSetOutputPortWidth(S, 5, 4);//dp_left
  ssSetOutputPortWidth(S, 6, 4);//ddp_left
  ssSetOutputPortWidth(S, 7, 4);//p_right
  ssSetOutputPortWidth(S, 8, 4);//dp_right
  ssSetOutputPortWidth(S, 9, 4);//ddp_right
  // Previewed motions:
  int nbsamples = 16;
  int nbsteps_max = 2;
  ssSetOutputPortWidth(S, 10, nbsteps_max);//first_foot_prw
  ssSetOutputPortWidth(S, 11, 4 * nbsamples);//com_prw (sample_instants, x, y, z)
  ssSetOutputPortWidth(S, 12, 3 * nbsamples);//cop_prw (sample_instants, x, y)

  ssSetOutputPortWidth(S, 13, 1);//support

  ssSetNumSampleTimes(S, 1);
  // Reserve place for C++ object
  ssSetNumPWork(S, 2);
  ssSetNumIWork(S, 1);

  //    ssSetOptions(S,
  //                 SS_OPTION_WORKS_WITH_CODE_REUSE |
  //                 SS_OPTION_EXCEPTION_FREE_CODE);

}


// Function: mdlInitializeSampleTimes =========================================
// Abstract:
//   This function is used to specify the sample time(s) for your
//   S-function. You must register the same number of sample times as
//   specified in ssSetNumSampleTimes.
static void mdlInitializeSampleTimes(SimStruct *S)
{
  ssSetSampleTime(S, 0, 0.001);
  //ssSetOffsetTime(S, 0, 0.0);
  //ssSetModelReferenceSampleTimeDefaultInheritance(S); 
}

// Function: mdlStart =======================================================
// Abstract:
//   This function is called once at start of model execution. If you
//   have states that should be initialized once, this is the place
//   to do it.
#define MDL_START
static void mdlStart(SimStruct *S)//TODO: mdlInitializeConditions if subsystem enabled
{

  WalkgenAbstract *walk = createWalkgen();
  ssSetPWorkValue(S, 0, (void*)walk);
  ssSetIWorkValue(S, 0, 0);//MPCWalkgen not initialized

}

// Function: mdlOutputs =======================================================
// Abstract:
//   In this function, you compute the outputs of your S-function
//   block. Generally outputs are placed in the output vector, ssGetY(S).
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

  WalkgenAbstract *walk = (WalkgenAbstract *)ssGetPWorkValue(S, 0);
  RigidBodySystem *robot = walk->robot();

  // Initialization:
  // ---------------
  if (ssGetIWorkValue(S, 0) == 0) {

    walk->reference(*vel_ref[0], *vel_ref[1], *vel_ref[2]);

    FootData leftFoot;// TODO: Never used
    double security_margin = 0.0;
    leftFoot.anklePositionInLocalFrame << 0, 0, 0; //0, 0, 0.105;//TODO:Is not used
    leftFoot.soleHeight = *foot_geometry[2] - *foot_geometry[3] - security_margin;
    leftFoot.soleWidth = *foot_geometry[0] - *foot_geometry[1] - security_margin;
    FootData rightFoot;
    rightFoot.anklePositionInLocalFrame << 0, 0, 0; //0, 0, 0.105;//TODO:Is not used
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

    MPCData mpcData;
    mpcData.nbsamples_qp = 16;
    mpcData.nbqpsamples_step = 8;
    mpcData.nbqpsamples_dsss = 8;
    mpcData.nbsteps_ssds = 2;
    mpcData.period_qpsample = 0.1;
    mpcData.period_mpcsample = 0.1;
    mpcData.period_actsample = 0.001;
    mpcData.ponderation.JerkMin[0] = 0.00001;
    mpcData.ponderation.JerkMin[1] = 0.00001;
    mpcData.warmstart = true;
    mpcData.interpolate_preview = true;
    mpcData.solver = QPOASES;


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

    walk->Init(robot_data, mpcData);
    ssSetIWorkValue(S, 0, 1);//Is initialized
  }

  // INPUT:
  // ------
  walk->reference(*vel_ref[0], *vel_ref[1], *vel_ref[2]);
  double gravity = 9.81;
  if (*closed_loop_in[0] > 0.5) {// TODO: Is there a better way for switching
    robot->com()->state().x[0] = *com_in[0];
    robot->com()->state().y[0] = *com_in[1];
    robot->com()->state().x[1] = *com_in[3];
    robot->com()->state().y[1] = *com_in[4];
    robot->com()->state().x[2] = gravity / *com_in[2] * (*com_in[0] - *cop_in[0]);
    robot->com()->state().y[2] = gravity / *com_in[2] * (*com_in[1] - *cop_in[1]);
  }

  // Run simulation:
  // ---------------
  double t = ssGetT(S);
  MPCSolution solution;
  solution = walk->online(t);

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

  // Previewed motions
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
    support[0] = current_support.time_limit - 0.1;// TODO: 0.1 is temporary solution
  } else {
    support[0] = 0.0;
  }


}

// Function: mdlTerminate =====================================================
// Abstract:
//   In this function, you should perform any actions that are necessary
//   at the termination of a simulation.  For example, if memory was
//   allocated in mdlStart, this is the place to free it.
static void mdlTerminate(SimStruct *S)
{
  WalkgenAbstract *walk = static_cast<WalkgenAbstract *>(ssGetPWork(S)[0]);
  delete walk;
}


// Required S-function trailer
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
