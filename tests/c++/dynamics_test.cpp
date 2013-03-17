#include <mpc-walkgen.h>

#include <iostream>

using namespace Eigen;
using namespace MPCWalkgen;

int main() {

	// Common simulation parameters:
	// -----------------------------
	MPCParameters mpc_parameters;
	mpc_parameters.num_samples_dsss       		= 13;
	mpc_parameters.num_steps_ssds         		= 2;
	mpc_parameters.num_steps_max				= 3;
	mpc_parameters.period_qpsample        		= .1;
	mpc_parameters.period_recomputation       	= .005;
	mpc_parameters.period_actsample       		= .005;
	mpc_parameters.period_ss					= 0.7;
	mpc_parameters.solver.name                  = QLD;
	mpc_parameters.dynamics_order               = SECOND_ORDER;

	mpc_parameters.penalties.cp[0] 				= 1.;
	mpc_parameters.penalties.cp[1] 				= 1.;

	// Robot parameters:
	// -----------------
	const double kSecurityMargin 	= .02;
	FootData left_foot;
	left_foot.ankle_pos_local 	<< -0.035, 0, 0.105;
	left_foot.sole_height 		= 0.095;
	left_foot.sole_width 		= 0.136;
	left_foot.position[0] 		= 0.;//-0.035;
	left_foot.position[1] 		= 0.22;
	left_foot.position[2] 		= 0.0;
	left_foot.SetEdges(0.13, -0.06, 0.0475, -0.0475, kSecurityMargin);

	FootData right_foot;
	right_foot.ankle_pos_local 	<< -0.035, 0, 0.105;
	right_foot.sole_height 		= 0.095;
	right_foot.sole_width 		= 0.136;
	right_foot.position[0] 		= 0.;//-0.035;
	right_foot.position[1] 		= -0.0059;
	right_foot.position[2] 		= 0.0;
	right_foot.SetEdges(0.13, -0.06, 0.0475, -0.0475, kSecurityMargin);

	HipYawData left_hip_yaw, right_hip_yaw;
	RobotData robot_data(left_foot, right_foot, left_hip_yaw, right_hip_yaw, 0.);

	robot_data.com(0) = 0.01;
	robot_data.com(1) = 0.01;
	robot_data.com(2) = 0.814;

	// Constants:
	const double kOmega = sqrt(kGravity / robot_data.com(2));
	const double kDeltaCP = 2.;




	// --------------------------------------
	// Create and initialize first generator:
	// --------------------------------------
	std::cout << std::endl;
	std::cout << "--------------------- " << std::endl;
	std::cout << "Standard formulation: " << std::endl;
	std::cout << "--------------------- " << std::endl;


	mpc_parameters.num_samples_horizon_max    	= 16;
	mpc_parameters.formulation		      		= STANDARD;
	mpc_parameters.num_samples_first_coarse_period	= 0;//5;
	mpc_parameters.num_samples_first_fine_period	= 0;//4;
	mpc_parameters.period_inter_samples			= 0;//0.02;


	Walkgen walk;
	walk.Init(mpc_parameters);
	walk.Init(robot_data);

	// Go:
	// ---
	walk.SetVelReference(0.1, 0., 0.);
	double curr_time = 0.;
	for (; curr_time < .105; curr_time += mpc_parameters.period_actsample ) {

		const MPCSolution &solution = walk.Go(curr_time);

		const MPCParameters &parameters = walk.mpc_parameters();
		const RigidBodySystem *robot = walk.robot();

		// Exponential CP reference:
		// -------------------------
		CommonVectorType cp_ref_vec = CommonVectorType::Zero(solution.sampling_times_vec.size() - 1);
		double time_passed = 0.;
		for (int i = 0; i < cp_ref_vec.rows(); i++) {
			time_passed = solution.sampling_times_vec[i + 1] - curr_time;
			cp_ref_vec(i) = robot_data.left_foot.position[Y] - exp(kOmega * time_passed) * kDeltaCP;
		}

		// Response of the system's unstable mode to a constant zmp:
		// ---------------------------------------------------------
		CommonVectorType contr_vec = CommonVectorType::Ones(parameters.num_samples_horizon) * robot_data.left_foot.position[Y];
		int mat_num = parameters.GetMPCSamplesLeft(solution.first_coarse_period);
		LinearDynamicsMatrices dyn_mat = robot->com()->dynamics_qp_vec()[mat_num].cp;
		CommonVectorType state = CommonVectorType::Zero(2);
		state(0) = robot_data.left_foot.position[Y] - kDeltaCP;
		CommonVectorType traj_vec = dyn_mat.state_mat * state + dyn_mat.input_mat * contr_vec;

		// Compare:
		// --------
		assert(cp_ref_vec.rows() == traj_vec.rows());

		CommonVectorType diff_vec = (traj_vec.array() - cp_ref_vec.array()) / traj_vec.array() * 100.;

		Debug::Cout("solution.sampling_times_vec: ", solution.sampling_times_vec);
		std::cout << "mat_num: " << mat_num << " Difference in %: " << diff_vec.transpose() << std::endl;
	}




	// ---------------------------------------
	// Create and initialize second generator:
	// ---------------------------------------
	std::cout << std::endl;
	std::cout << "---------------- " << std::endl;
	std::cout << "Decoupled modes: " << std::endl;
	std::cout << "---------------- " << std::endl;

	mpc_parameters.num_samples_horizon_max    		= 24;
	mpc_parameters.formulation		      			= DECOUPLED_MODES;
	mpc_parameters.num_samples_first_fine_period	= 4;
	mpc_parameters.num_samples_first_coarse_period	= 5;
	mpc_parameters.period_inter_samples				= 0.02;

	Walkgen walk2;
	walk2.Init(mpc_parameters);
	walk2.Init(robot_data);

	// Go:
	// ---
	walk2.SetVelReference(0.1, 0., 0.);
	curr_time = 0.;
	for (; curr_time < .105; curr_time += mpc_parameters.period_actsample ) {

		const MPCSolution &solution = walk2.Go(curr_time);

		const MPCParameters &parameters = walk2.mpc_parameters();
		const RigidBodySystem *robot = walk2.robot();

		// Exponential CP reference:
		// -------------------------
		CommonVectorType cp_ref_vec = CommonVectorType::Zero(solution.sampling_times_vec.size() - 1);
		double time_passed = 0.;
		for (int i = 0; i < cp_ref_vec.rows(); i++) {
			time_passed = solution.sampling_times_vec[i + 1] - curr_time;
			cp_ref_vec(i) = robot_data.left_foot.position[Y] - exp(kOmega * time_passed) * kDeltaCP;
		}

		// Response of the system's unstable mode to a constant zmp:
		// ---------------------------------------------------------
		CommonVectorType contr_vec = CommonVectorType::Ones(parameters.num_samples_horizon + 1) * robot_data.left_foot.position[Y];
		int mat_num = parameters.GetMPCSamplesLeft(solution.first_coarse_period);
		LinearDynamicsMatrices dyn_mat = robot->com()->dynamics_qp_vec()[mat_num].cp;
		Matrix2D state_trans_mat = Matrix2D::Zero();
		state_trans_mat(0, 0) = 1.;
		state_trans_mat(0, 1) = -1. / kOmega;
		state_trans_mat(1, 0) = 1.;
		state_trans_mat(1, 1) = 1. / kOmega;
		CommonVectorType orig_state = CommonVectorType::Zero(2);
		orig_state(0) = robot_data.left_foot.position[Y] - kDeltaCP;
		CommonVectorType new_state = state_trans_mat * orig_state;
		CommonVectorType stab_state = new_state.block(0, 0, 1, 1);
		contr_vec.tail(1) = cp_ref_vec.tail(1);
		CommonVectorType traj_vec = dyn_mat.state_mat * stab_state + dyn_mat.input_mat * contr_vec;

		// Compare:
		// --------
		assert(cp_ref_vec.rows() == traj_vec.rows());

		CommonVectorType diff_vec = (traj_vec.array() - cp_ref_vec.array()) / traj_vec.array() * 100.;

		Debug::Cout("solution.sampling_times_vec: ", solution.sampling_times_vec);
		std::cout << "mat_num: " << mat_num << " Difference in %: " << diff_vec.transpose() << std::endl;
	}

	return 0;
}
