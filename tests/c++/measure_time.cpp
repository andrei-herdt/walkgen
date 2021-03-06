#include <mpc-walkgen.h>

#include <iostream>

using namespace Eigen;
using namespace MPCWalkgen;

int main() {

	double security_margin 		= .02;

	int num_samples_horizon_max 		= 16;
	int num_steps_ssds 			= 2;
	// Simulation parameters:
	// ----------------------
	MPCParameters mpc_parameters;
	mpc_parameters.num_samples_horizon_max    	= num_samples_horizon_max;
	mpc_parameters.num_samples_first_coarse_period	= 5;
	mpc_parameters.num_samples_first_fine_period	= 4;
	mpc_parameters.num_samples_state		= 160;
	mpc_parameters.num_samples_contr		= num_samples_horizon_max;
	mpc_parameters.period_dsss       		= 0.8;
	mpc_parameters.num_steps_ssds         		= std::min(num_steps_ssds, num_samples_horizon_max);
	mpc_parameters.num_steps_max			= 3;
	mpc_parameters.period_qpsample        		= .1;
	mpc_parameters.period_inter_samples		= 0.02;
	mpc_parameters.period_recomputation       	= .005;
	mpc_parameters.period_actsample       		= .005;
	mpc_parameters.period_ss			= 0.7;
	mpc_parameters.warmstart           		= false;
	mpc_parameters.interpolate_whole_horizon    	= false;
	mpc_parameters.solver.analysis              	= false;
	mpc_parameters.problem_dumping		      	= false;
	mpc_parameters.solver.name                  	= QLD;
	mpc_parameters.solver.num_wsrec             	= 100;
	mpc_parameters.dynamics_order               	= SECOND_ORDER;
	mpc_parameters.formulation		      	= DECOUPLED_MODES;
	mpc_parameters.mapping			      	= CONST_MAP;
	mpc_parameters.is_pid_mode		      	= false;
	mpc_parameters.is_terminal_constr	      	= false;
	mpc_parameters.is_ineq_constr			= false;
	mpc_parameters.problem_dumping			= false;

	mpc_parameters.penalties.pos[0] 		= 0.;
	mpc_parameters.penalties.vel[0]  		= 0.;
	mpc_parameters.penalties.cop[0]  		= 0.02;
	mpc_parameters.penalties.contr_moves[0] 	= 0;
	mpc_parameters.penalties.cp[0] 			= 1.;
	mpc_parameters.penalties.first_contr_moves 	= 0;
	mpc_parameters.penalties.cp_fp[0] 		= 0.;
	mpc_parameters.penalties.second_contr_moves 	= 0;

	mpc_parameters.penalties.pos[1] 		= 0.;
	mpc_parameters.penalties.vel[1]  		= 0.;
	mpc_parameters.penalties.cop[1]  		= 0.02;
	mpc_parameters.penalties.cp[1] 			= 1.;
	mpc_parameters.penalties.cp_fp[1] 		= 0.;
	mpc_parameters.penalties.contr_moves[1] 	= 0.;

	mpc_parameters.init_com_height 			= 0;

	// Robot parameters:
	// -----------------
	FootData left_foot;
	left_foot.ankle_pos_local 	<< -0.035, 0, 0.105;
	left_foot.sole_height 		= 0.095;
	left_foot.sole_width 		= 0.136;
	left_foot.position[0] 		= 0.;//-0.035;
	left_foot.position[1] 		= 0.22;
	left_foot.position[2] 		= 0.0;
	left_foot.SetEdges(0.13, -0.06, 0.0475, -0.0475, security_margin, security_margin, security_margin, security_margin);

	FootData right_foot;
	right_foot.ankle_pos_local 	<< -0.035, 0, 0.105;
	right_foot.sole_height 		= 0.095;
	right_foot.sole_width 		= 0.136;
	right_foot.position[0] 		= 0.;//-0.035;
	right_foot.position[1] 		= -0.0059;
	right_foot.position[2] 		= 0.0;
	right_foot.SetEdges(0.13, -0.06, 0.0475, -0.0475, security_margin, security_margin, security_margin, security_margin);


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
	robot_data.com(1) = 0.01;
	robot_data.com(2) = 0.814;

	robot_data.max_foot_vel = 1.;

	// Feasibility hulls:
	// ------------------
	const int num_vertices = 5;
	// Feasible foot positions
	double foot_pos_vertices_x[num_vertices] = {-0.2, -0.2, 0.0, 0.2, 0.2};
	double foot_pos_vertices_y[num_vertices] = {-0.2, -0.3, -0.4, -0.3, -0.225};

	robot_data.left_foot_pos_hull.Resize(num_vertices);
	robot_data.right_foot_pos_hull.Resize(num_vertices);
	for (int i=0; i < num_vertices; ++i) {
		robot_data.left_foot_pos_hull.x_vec(i) = foot_pos_vertices_x[i];
		robot_data.left_foot_pos_hull.y_vec(i) = foot_pos_vertices_y[i];
		robot_data.right_foot_pos_hull.x_vec(i) = foot_pos_vertices_x[i];
		robot_data.right_foot_pos_hull.y_vec(i) = -foot_pos_vertices_y[i];
	}

	double feet_distance_y = left_foot.position[1] - right_foot.position[1];
	robot_data.SetCoPHulls(feet_distance_y);

	// Create and initialize generator:
	// -------------------------------
	Walkgen walk;
	walk.Init(mpc_parameters);
	walk.Init(robot_data);

	// Go:
	// ---
	double curr_time = 0.;
	walk.SetVelReference(0.1, 0., 0.);
	int num_iterations = 0;
	walk.clock().GetFrequency(1000);
	walk.clock().ReserveMemory(20, 2000);

	walk.clock().ResetLocal();
	for (; curr_time < 10.; curr_time += mpc_parameters.period_actsample) {
                walk.SetCPReference(0., 0., 0., 0.0145);
		int online_timer = walk.clock().StartCounter();
		//std::cout << std::endl;
		//std::cout << std::endl;
		//std::cout << std::endl;
		//std::cout<< "curr_time: " << curr_time << std::endl;
		//std::cout << "--------------------" << std::endl;
		const MPCSolution &solution = walk.Go(curr_time);
		//walk.solver()->DumpMatrices(curr_time, "dat");
		
		//Debug::Cout("com_prw.pos.x", solution.com_prw.pos.x_vec);
		//Debug::Cout("com_prw.vel.x", solution.com_prw.vel.x_vec);
		//std::cout << "com_prw.vel: " << solution.com_prw.vel.x_vec.transpose() << std::endl;
		
		//std::cout << "com_prw.cp.x: " << solution.com_prw.cp.x_vec.transpose() << std::endl;
		//std::cout << "com_prw.cp.x: " << solution.com_prw.cp.x_vec.transpose() << std::endl;
	 	
		//std::cout << std::endl;
		//std::cout << "com_prw.control.x_vec: " << solution.com_prw.control.x_vec.transpose() << std::endl;
	 	//std::cout << std::endl;
		//Debug::Cout("solution.qp_solution_vec: ", solution.qp_solution_vec);
		//Debug::Cout("com_prw.control.y_vec", solution.com_prw.control.y_vec);
	 	//std::cout << std::endl;
		//std::cout << "com_act.pos.x: " << walk.output().com.x << "  com_act.vel.x: " << walk.output().com.dx << std::endl;
		
		//Debug::Cout("sampling_times_vec", solution.sampling_times_vec);
		//Debug::WriteToDatFile("hessian", curr_time, walk.solver()->hessian_mat()());
		//Debug::WriteToDatFile("gradient", curr_time, walk.solver()->gradient_vec()());
		//walk.clock().StopLastCounter();
		walk.clock().StopCounter(online_timer);
		walk.clock().ResetLocal();
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

	Debug::Cout("Distribution of ticks: ", walk.clock().ticks_distr_vec());

	return 0;
}
