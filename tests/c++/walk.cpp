// This test has been written for the terminal based analysis 
// of the computation time
// Author: Andrei Herdt

#include <mpc-walkgen.h>

#include <iostream>

using namespace Eigen;
using namespace MPCWalkgen;

int main() {

	int num_samples_horizon 		= 16;
	int num_samples_step 			= 5;
	int num_samples_dsss 			= 5;
	int num_steps_ssds 				= 2;
	double sample_period_qp 		= .1;
	double sample_period_first 		= .005;
	double sample_period_act 		= .005;
	const double kSecurityMargin 	= .02;

	// Simulation parameters:
	// ----------------------
	MPCParameters mpc_parameters;
	mpc_parameters.num_samples_horizon    = num_samples_horizon;
	mpc_parameters.num_samples_step       = std::min(num_samples_step, num_samples_horizon);
	mpc_parameters.num_samples_dsss       = std::min(num_samples_dsss, num_samples_horizon);
	mpc_parameters.num_steps_ssds         = std::min(num_steps_ssds, num_samples_horizon);
	mpc_parameters.period_qpsample        = sample_period_qp;
	mpc_parameters.period_mpcsample       = sample_period_first;
	mpc_parameters.period_actsample       = sample_period_act;
	mpc_parameters.warmstart                      = false;
	mpc_parameters.interpolate_whole_horizon      = true;
	mpc_parameters.solver.analysis                = false;
	mpc_parameters.solver.name                    = QPOASES;
	mpc_parameters.solver.num_wsrec               = 200;
	mpc_parameters.dynamics_order                 = SECOND_ORDER;
	mpc_parameters.formulation		      = DECOUPLED_MODES;
	mpc_parameters.is_pid_mode		      = false;

	mpc_parameters.weights.pos[0] 		= 0.;
	mpc_parameters.weights.vel[0]  		= 0.;
	mpc_parameters.weights.cop[0]  		= 0.;//0.00001;
	mpc_parameters.weights.cp[0] 		= 1.;//1.;
	mpc_parameters.weights.control[0] 	= .00001;

	mpc_parameters.weights.pos[1] 		= 0.;
	mpc_parameters.weights.vel[1]  		= 0.;
	mpc_parameters.weights.cop[1]  		= 0.;//1.;
	mpc_parameters.weights.cp[1] 		= 1.;
	mpc_parameters.weights.control[1] 	= .00001;

	// Robot parameters:
	// -----------------
	FootData left_foot;
	left_foot.ankle_pos_local 	<< 0, 0, 0.105;
	left_foot.sole_height 		= 0.138;
	left_foot.sole_width 		= 0.2172;
	left_foot.position[0] 		= 0.00949035;
	left_foot.position[1] 		= 0.095;
	left_foot.position[2] 		= 0.0;
	left_foot.SetEdges(0.2172, 0.0, 0.138, 0.0, kSecurityMargin);

	FootData right_foot;
	right_foot.ankle_pos_local 	<< 0, 0, 0.105;
	right_foot.sole_height 		= 0.138;
	right_foot.sole_width 		= 0.2172;
	right_foot.position[0] 		= 0.00949035;
	right_foot.position[1] 		= -0.095;
	right_foot.position[2] 		= 0.0;
	right_foot.SetEdges(0.2172, 0.0, 0.138, 0.0, kSecurityMargin);


	HipYawData left_hip_yaw;
	left_hip_yaw.lower_pos_bound = -0.523599;
	left_hip_yaw.upper_pos_bound = 0.785398;
	left_hip_yaw.lower_vel_bound = -3.54108;
	left_hip_yaw.upper_vel_bound = 3.54108;
	left_hip_yaw.lower_acc_bound = -0.1;
	left_hip_yaw.upper_acc_bound = 0.1;
	HipYawData right_hip_yaw = left_hip_yaw;

	RobotData robot_data(left_foot, right_foot, left_hip_yaw, right_hip_yaw, 0.);

	robot_data.com(0) = 0.01;
	robot_data.com(1) = 0.05;
	robot_data.com(2) = 0.814;

	robot_data.max_foot_vel = 1.;

	// Feasibility hulls:
	// ------------------
	const int num_vertices = 5;
	// Feasible foot positions
	double foot_pos_vertices_x[num_vertices] = {-0.28, -0.2, 0.0, 0.2, 0.28};
	double foot_pos_vertices_y[num_vertices] = {-0.2, -0.3, -0.4, -0.3, -0.2};

	robot_data.left_foot_pos_hull.Resize(num_vertices);
	robot_data.right_foot_pos_hull.Resize(num_vertices);
	for (int i=0; i < num_vertices; ++i) {
		robot_data.left_foot_pos_hull.x_vec(i) = foot_pos_vertices_x[i];
		robot_data.left_foot_pos_hull.y_vec(i) = foot_pos_vertices_y[i];
		robot_data.right_foot_pos_hull.x_vec(i) = foot_pos_vertices_x[i];
		robot_data.right_foot_pos_hull.y_vec(i) = -foot_pos_vertices_y[i];
	}


	// Constraints on the CoP
	const int num_vertices_cop = 4;
	double cop_vertices_ss_x[num_vertices_cop] = {0.0686, 0.0686, -0.0686, -0.0686};
	double cop_vertices_ss_y[num_vertices_cop] = {0.029, -0.029, -0.029, 0.029};
	double cop_vertices_ds_x[num_vertices_cop] = {0.0686, 0.0686, -0.0686, -0.0686};
	double cop_vertices_ds_y[num_vertices_cop] = {0.029, -0.229, -0.229, 0.029};

	robot_data.left_foot_ss_hull.Resize(num_vertices_cop);
	robot_data.right_foot_ss_hull.Resize(num_vertices_cop);
	robot_data.left_foot_ds_hull.Resize(num_vertices_cop);
	robot_data.right_foot_ds_hull.Resize(num_vertices_cop);
	for (int i = 0; i < num_vertices_cop; ++i) {
		robot_data.left_foot_ss_hull.x_vec(i) = cop_vertices_ss_x[i];
		robot_data.left_foot_ss_hull.y_vec(i) = cop_vertices_ss_y[i];
		robot_data.left_foot_ds_hull.x_vec(i) = cop_vertices_ds_x[i];
		robot_data.left_foot_ds_hull.y_vec(i) = cop_vertices_ds_y[i];

		robot_data.right_foot_ss_hull.x_vec(i) = cop_vertices_ss_x[i];
		robot_data.right_foot_ss_hull.y_vec(i) =- cop_vertices_ss_y[i];
		robot_data.right_foot_ds_hull.x_vec(i) = cop_vertices_ds_x[i];
		robot_data.right_foot_ds_hull.y_vec(i) =- cop_vertices_ds_y[i];
	}


	// Create and initialize generator:
	// -------------------------------
	Walkgen walk;
	walk.Init(mpc_parameters);
	walk.Init(robot_data);
	//walk.robot()->com()->state().x[1] = .1;

	// Go:
	// ---
	double curr_time = 0.;
	walk.SetVelReference(0., 0., 0.);
	int num_iterations = 0;
	//walk.clock().GetFrequency(1000);
	//walk.clock().ResetLocal();
	for (; curr_time < .1; curr_time += sample_period_act) {
		//int online_timer = walk.clock().StartCounter();
		//std::cout << std::endl;
		const MPCSolution &solution = walk.Go(curr_time);
		walk.solver()->DumpMatrices(curr_time, "dat");
		//walk.solver()->DumpProblem("problem", curr_time, "txt");
		//std::cout << "com_prw.pos.x: " << solution.com_prw.pos.x_vec.transpose() << std::endl;
		//std::cout << "com_prw.pos.y: " << solution.com_prw.pos.y_vec.transpose() << std::endl;
		//std::cout << "com_prw.vel: " << solution.com_prw.vel.x_vec.transpose() << std::endl;
		//std::cout << "com_prw.cp.x: " << solution.com_prw.cp.x_vec.transpose() << std::endl;
		//std::cout << "com_prw.cop.x_vec: " << solution.com_prw.cop.x_vec.transpose() << std::endl;
		//std::cout << "com_act.pos: " << walk.output().com.x << "  com_act.vel: " << walk.output().com.dx << std::endl;
		//Debug::Cout("sampling_times_vec", solution.sampling_times_vec);
		//Debug::WriteToDatFile("hessian", curr_time, walk.solver()->hessian_mat()());
		//Debug::WriteToDatFile("gradient", curr_time, walk.solver()->gradient_vec()());
		//walk.clock().StopLastCounter();
		//walk.clock().StopCounter(online_timer);
		//walk.clock().ResetLocal();
		num_iterations++;
	}
		walk.solver()->DumpMatrices(curr_time, "dat");
	/*
	walk.SetVelReference(0., 0., 0.);
	for (; curr_time < 20; curr_time += sample_period_act) {
		int online_timer = walk.clock().StartCounter();
		const MPCSolution &solution = walk.Go(curr_time);
		//walk.clock().StopLastCounter();
		walk.clock().StopCounter(online_timer);
		walk.clock().ResetLocal();
		num_iterations++;
	}
	walk.SetVelReference(0., 0, 0.);
	for (; curr_time < 25; curr_time += sample_period_act) {
		int online_timer = walk.clock().StartCounter();
		const MPCSolution &solution = walk.Go(curr_time);
		//walk.clock().StopLastCounter();
		walk.clock().StopCounter(online_timer);
		walk.clock().ResetLocal();
		num_iterations++;
	}
	*/


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

	// Print max time:
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
