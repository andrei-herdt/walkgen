// This test has been written for the terminal based analysis 
// of the computation time
// Author: Andrei Herdt

#include <mpc-walkgen.h>

#include <iostream>

using namespace Eigen;
using namespace MPCWalkgen;

int main() {

  int num_samples_horizon = 10;
  int num_samples_step = 8;
  int num_samples_dsss = 8;
  int num_steps_ssds = 2;
  double sample_period_qp = 0.1;
  double sample_period_first = 0.002;
  double sample_period_act = 0.002;
  const double kSecurityMargin = 0.02;

    // Simulation parameters:
  // ----------------------
  MPCData mpc_parameters;
  mpc_parameters.nbsamples_qp       = num_samples_horizon;
  mpc_parameters.nbqpsamples_step   = num_samples_step;
  mpc_parameters.nbqpsamples_dsss   = num_samples_dsss;
  mpc_parameters.nbsteps_ssds       = num_steps_ssds;
  mpc_parameters.period_qpsample    = sample_period_qp;
  mpc_parameters.period_mpcsample   = sample_period_first;
  mpc_parameters.period_actsample   = sample_period_act;
  mpc_parameters.weight_coefficients.jerk[0]         = 0.001;
  mpc_parameters.weight_coefficients.jerk[1]         = 0.001;
  mpc_parameters.warmstart                      = false;
  mpc_parameters.interpolate_whole_horizon      = false;
  mpc_parameters.solver.analysis                = false;
  mpc_parameters.solver.name                    = QPOASES;
  mpc_parameters.solver.num_wsrec               = 2;
  mpc_parameters.dynamics_order                 = SECOND_ORDER;



  // Robot parameters:
  // -----------------
  FootData leftFoot;
  leftFoot.anklePositionInLocalFrame      << 0, 0, 0.105;
  leftFoot.soleHeight                     = 0.138;
  leftFoot.soleWidth                      = 0.2172;
  leftFoot.position[0] = 0.00949035;
  leftFoot.position[1] = 0.095;
  leftFoot.position[2] = 0.0;
  leftFoot.SetEdges(0.2172, 0.0, 0.138, 0.0, kSecurityMargin);    

  FootData rightFoot;
  rightFoot.anklePositionInLocalFrame     << 0, 0, 0.105;
  rightFoot.soleHeight                    = 0.138;
  rightFoot.soleWidth                     = 0.2172;
  rightFoot.position[0] = 0.00949035;
  rightFoot.position[1] = -0.095;
  rightFoot.position[2] = 0.0;
  rightFoot.SetEdges(0.2172, 0.0, 0.138, 0.0, kSecurityMargin);    


  HipYawData leftHipYaw;
  leftHipYaw.lowerBound                   = -0.523599;
  leftHipYaw.upperBound                   = 0.785398;
  leftHipYaw.lowerVelocityBound           = -3.54108;
  leftHipYaw.upperVelocityBound           = 3.54108;
  leftHipYaw.lowerAccelerationBound       = -0.1;
  leftHipYaw.upperAccelerationBound       = 0.1;
  HipYawData rightHipYaw = leftHipYaw;

  RobotData robot_data(leftFoot, rightFoot, leftHipYaw, rightHipYaw, 0.0);

    // TODO: This initialization did not work
    robot_data.com(0) = 0.0;
    robot_data.com(1) = 0.0;
    robot_data.com(2) = 0.814;


  // Feasible hulls:
  // ---------------
  const int nbVertFeet = 5;
  // Feasible foot positions
  double DefaultFPosEdgesX[nbVertFeet] = {-0.28, -0.2, 0.0, 0.2, 0.28};
  double DefaultFPosEdgesY[nbVertFeet] = {-0.2, -0.3, -0.4, -0.3, -0.2};

  robot_data.leftFootHull.resize(nbVertFeet);
  robot_data.rightFootHull.resize(nbVertFeet);
  for (int i=0; i < nbVertFeet; ++i) {
    robot_data.leftFootHull.x(i) = DefaultFPosEdgesX[i];
    robot_data.leftFootHull.y(i) = DefaultFPosEdgesY[i];
    robot_data.rightFootHull.x(i) = DefaultFPosEdgesX[i];
    robot_data.rightFootHull.y(i) = -DefaultFPosEdgesY[i];
  }


  // Constraints on the CoP
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


  // Create and initialize generator:
  // -------------------------------
  Walkgen walk;
  walk.Init(mpc_parameters);
  walk.Init(robot_data);

  // Go:
  // ---
  double velocity = 0.1;
  double curr_time = 0;
  walk.reference(0.1, 0, 0);
  int num_iterations = 0;
  walk.clock().GetFrequency(1000);
  for (; curr_time < 20; curr_time += sample_period_act) {
    walk.clock().ResetLocal();
    int online_timer = walk.clock().StartCounter();
    const MPCSolution &solution = walk.online(curr_time);
    walk.clock().StopLastCounter();
    walk.clock().StopCounter(online_timer);
    
    num_iterations++;
  }


  // Print total time:
  int num_counters = walk.clock().GetNumTotalCounters();
  double passed_time = 0.0;
  std::cout << "Total [mus]: ----------------" << std::endl;
  for (int counter = num_counters - 1; counter >= 1; counter--) {
    passed_time = walk.clock().GetTotalTime(counter);
    std::cout<<"num"<< counter <<": " << passed_time << "   ";
  }
  passed_time = walk.clock().GetTotalTime(0);
  std::cout << "total: " << passed_time  << std::endl;

  // Print mean time:
  std::cout << "Mean [mus]: ----------------" << std::endl;
  for (int counter = num_counters - 1; counter >= 1; counter--) {
    passed_time = walk.clock().GetTotalTime(counter) / num_iterations;
    std::cout<<"num"<< counter <<": " << passed_time << "   ";
  }
  passed_time = walk.clock().GetTotalTime(0) / num_iterations;
  std::cout << "total: " << passed_time  << std::endl;

  // Print mean time:
  std::cout << "Max [mus]: ----------------" << std::endl;
  for (int counter = num_counters - 1; counter >= 1; counter--) {
    passed_time = walk.clock().GetMaxTime(counter);
    std::cout<<"num"<< counter <<": " << passed_time << "   ";
  }
  passed_time = walk.clock().GetMaxTime(0);
  std::cout << "total: " << passed_time  << std::endl;

  std::cout << "Final CoM position: " << walk.output().com.x <<
    ", " << walk.output().com.y <<std::endl;

  return 0;
}