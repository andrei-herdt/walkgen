#pragma once
#ifndef MPC_WALKGEN_TYPES_H
#define MPC_WALKGEN_TYPES_H

#include <mpc-walkgen/api.h>

#include <Eigen/Dense>
#include <vector>

namespace MPCWalkgen{

inline double round( double d )
{	return floor( d + 0.5 );	}

//
// Constants
//
const static double kEps = 1e-9;
const static double kGravity = 9.81;
const static double kInf = 1e10;

//
// Enums:
//
enum HullType{ FOOT_HULL, COP_HULL };//TODO: Remove this

enum Derivative {
	POSITION		= 0,
	VELOCITY		= 1,
	ACCELERATION	= 2,
	JERK			= 3,
	COP				= 4
};

enum SampleRate { QP, ACTUATORS };//TODO: Needed?

enum Phase { SS, DS };

enum Foot { LEFT, RIGHT };

enum BodyType { LEFT_FOOT, RIGHT_FOOT, COM  };

enum SolverName { QPOASES, LSSOL, QLD };

enum Axis { X, Y, Z, Yaw };

enum SystemOrder { FIRST_ORDER = 1, SECOND_ORDER = 2, THIRD_ORDER = 3 };

enum Mode { INITIAL, WALK, STOP};

enum Formulation {
	STANDARD = 0,			//State is directly the state of the particle
	DECOUPLED_MODES = 1		//Stable and unstable modes are decoupled
};

//
// Typedefs:
//
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixRowMaj;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> MatrixColMaj;
typedef Eigen::Matrix2d Matrix2D;
typedef MatrixRowMaj CommonMatrixType;

typedef Eigen::Vector2d Vector2D;
typedef Eigen::Vector3d Vector3D;
typedef Eigen::VectorXd CommonVectorType;

typedef unsigned int uint;

//
// Data structures:
//
struct Frame{
	CommonVectorType x, y, yaw;

	Frame();

	void SetZero(int size);
};

struct Reference{
	Frame global, local;

	Reference();

	void SetZero(int size);
};

struct MPC_WALKGEN_API BodyState{
	Vector3D x, y, z;
	Vector3D yaw, pitch, roll;

	BodyState();

	void Reset();
};

struct Wrench {
	double force_x, force_y, force_z;
	double torque_x, torque_y, torque_z;

	void SetZero();
};

struct MPC_WALKGEN_API FootData{
	double sole_width;
	double sole_height;

	Vector3D ankle_pos_local;

	std::vector<double> edges_x_vec;
	std::vector<double> edges_y_vec;

	std::vector<double> position;

	FootData();
	FootData(const FootData &f);//TODO: LocalAnklePosition_ better?
	~FootData();

	void SetEdges(double front, double back, double left, double right, double security_margin);
};

struct MPC_WALKGEN_API HipYawData {
	double lower_pos_bound, upper_pos_bound;
	double lower_vel_bound, upper_vel_bound;
	double lower_acc_bound, upper_acc_bound;

	HipYawData();
	~HipYawData();
};

struct MPC_WALKGEN_API Penalties{
	std::vector<double> pos, vel, acc, jerk;

	std::vector<double> cop;
	std::vector<double> cp;			//Capture point
	std::vector<double> cp_fp;
	std::vector<double> contr_moves;
	double first_contr_move;

	/// \brief Define the element of weight_coefficients std::vector used in this iteration
	int active_mode;
	bool is_initial_mode;

	void SetCoefficients(Reference &ref);

	Penalties(int num_modes = 2);
	~Penalties();
};

struct MPC_WALKGEN_API SupportState {
	Phase phase;
	Foot foot;

	int num_steps_left;
	int step_number;
	int num_instants;

	double time_limit;
	double start_time;

	double x, y, yaw;

	bool state_changed;

	bool transitional_ds;	/// \brief Define if the support state is in a (transitional) double support phase

	double previous_sampling_period;	/// \brief The length of the previous sampling period (can be different from period_qpsample)
};

struct MPC_WALKGEN_API ConvexHull {
	/// \brief Set of vertices
	CommonVectorType x_vec;
	CommonVectorType y_vec;
	CommonVectorType z_vec;

	/// \brief Set of inequalities A*x + B*y + C*z + D>0
	CommonVectorType a_vec;
	CommonVectorType b_vec;
	CommonVectorType c_vec;
	CommonVectorType d_vec;

	ConvexHull();
	~ConvexHull();

	ConvexHull &operator= (const ConvexHull &hull); // TODO: CopyFrom() instead of =
	void Resize(int size);
	void RotateVertices(double yaw);
	void BuildInequalities(const Foot &foot);
};

struct SolverData {
	SolverName name;
	int num_wsrec;  			//Maximal number of working set recomputations
	bool analysis;
	bool dump_problem;
};

struct MPC_WALKGEN_API MPCParameters {
	double period_qpsample;				// Sampling period [sec]
	double period_inter_samples;		//
	double period_recomputation;			// Time between recomputations [sec]
	double period_actsample;			// Actuator sampling period [sec]
	double period_ds;					// Length of the (permanent) double support phase (should be a large value)
	double period_dsss;
	double period_ss;					// Length of single support phase

	double ds_force_thresh;				// Force that defines the double support phase
	double ffoot_plan_period;			// Time for planning the foot placement

	double init_com_height;				// Initial CoM height is given

	int num_samples_horizon_max;		// Number max of samples
	int num_samples_horizon;  			// Number of samplings inside horizon
	int num_samples_first_period;		// Number of additional samples inside first sampling period
	int num_samples_step;				// Step period ss_left<->ss_right in qp sample periods
	int num_samples_dsss;				// Length of initial double support phase [num. samples]
	int num_steps_ssds;					// Steps before halt
	int num_steps_max;

	bool warmstart;
	bool interpolate_whole_horizon;		// Interpolate not only the control (first element) but for the whole preview period
	bool is_closed_loop;
	bool is_pid_mode;
	bool is_ineq_constr;				// Turns on all inequality constraints
	bool is_terminal_constr;			// Turns on terminal constraints
	bool problem_dumping;				// Optimization program is written to file before being solved

	Mode walking_mode;	//TODO: Replace this.

	SystemOrder dynamics_order;

	Formulation formulation;

	Penalties penalties;

	SolverData solver;

	/// \brief Compute the number of recomputations left until next sample
	int GetMPCSamplesLeft(double first_sampling_period) const;
	/// \brief Number of simulation iterations between two feedback call
	int num_samples_act() const;

	int GetNumRecomputations() const;			/// \brief Number of feedback iterations between two QP instants

	int num_qpsamples_ss() const;

	double period_trans_ds() const;

	MPCParameters();
	~MPCParameters();
};

struct MPC_WALKGEN_API RobotData {
	double mass;
	double max_foot_height;
	double max_foot_vel;
	double security_margin;
	double lateral_ds_feet_dist;

	Vector3D com;

	FootData left_foot;
	FootData right_foot;

	HipYawData left_hip_yaw;
	HipYawData right_hip_yaw;

	ConvexHull left_foot_pos_hull;
	ConvexHull right_foot_pos_hull;
	ConvexHull left_foot_ss_hull;
	ConvexHull right_foot_ss_hull;
	ConvexHull left_foot_ds_hull;
	ConvexHull right_foot_ds_hull;

	RobotData(const FootData &left_foot, const FootData &right_foot,
			const HipYawData &leftHipYaw, const HipYawData &rightHipYaw,
			double mass);
	RobotData();
	~RobotData();

	void SetCoPHulls(double ds_distance);
	//TODO: void ComputeMaxFootVel();
};

struct MPC_WALKGEN_API Trajectory {
	CommonVectorType x_vec, y_vec, z_vec, yaw_vec;

	void SetZero(int size);
};

struct Motion {
	Trajectory pos, vel, acc, jerk;
	Trajectory control;
	Trajectory cp;
	Trajectory cop;

	void SetZero(int num_samples, int num_unst_modes);
};

struct SolutionAnalysis {
	int num_iterations;

	double objective_value;
	double resolution_time;
	double max_kkt_violation; //Maximum violation of the KKT optimality conditions

	SolutionAnalysis();
};


struct MPC_WALKGEN_API MPCSolution {
	CommonVectorType qp_solution_vec;
	CommonVectorType initial_solution;

	Eigen::VectorXi constraints;
	Eigen::VectorXi init_active_set;

	std::vector<double> sampling_times_vec;	/// Starting with the current time, i.e. all instants are global

	std::vector<SupportState> support_states_vec;

	std::vector<double> support_yaw_vec;
	std::vector<double> trunk_yaw_vec;

	Motion com_prw, left_foot_prw, right_foot_prw;
	Motion com_act, left_foot_act, right_foot_act;

	SolutionAnalysis analysis;

	Reference pos_ref;

	bool is_prev_sol_exist;

	double prev_first_foot_x, prev_first_foot_y;
	double prev_cop_x, prev_cop_y;

	double first_coarse_period;


	MPCSolution& operator = (MPCSolution const &);	//This operator helps to avoid alignment errors

	void Reset();

	MPCSolution();
	~MPCSolution();
};

struct MPC_WALKGEN_API StateValues {
	double x, 	dx, 	ddx;
	double y, 	dy, 	ddy;
	double z, 	dz, 	ddz;
	double yaw, dyaw, 	ddyaw;
};

struct MPC_WALKGEN_API ControlOutput {
	StateValues com, cop, left_foot, right_foot;
};

struct LinearDynamicsMatrices{
	// \name Prediction matrices
	// \{
	CommonMatrixType state_mat;
	CommonMatrixType state_mat_inv;
	CommonMatrixType stab_state_mat;		//Stable modes state matrix
	CommonMatrixType unst_state_mat;		//Unstable modes state matrix

	CommonMatrixType input_mat;
	CommonMatrixType input_mat_tr;
	CommonMatrixType input_mat_inv;
	CommonMatrixType input_mat_inv_tr;
	// \}

	// \name State-space dynamics matrices
	// \{
	CommonMatrixType c_state_mat;			//Continuous state matrix
	CommonMatrixType c_state_mat_inv;		//Inverse of continuous state matrix
	CommonMatrixType d_state_mat;			//Discrete state matrix
	CommonMatrixType d_state_mat_inv;		//Inverse of discrete state matrix

	CommonMatrixType c_input_mat;			//Continuous input matrix
	CommonMatrixType c_input_mat_tr;		//Transpose of continuous input matrix
	CommonMatrixType d_input_mat;			//Discrete input matrix
	CommonMatrixType d_input_mat_tr;		//Transpose of discrete input matrix

	CommonMatrixType ss_output_mat;			//Output matrix
	CommonMatrixType ss_output_mat_tr;		//Transpose of output matrix
	CommonMatrixType ss_feedthrough_mat;	//Feedthrough matrix
	// \}

	void SetZero(int state_dim,
			int input_dim,
			int output_dim,
			int num_samples,
			int stable_dim,			//Number of stable modes
			int unstable_dim		//Number of unstable modes
	);
};

struct LinearDynamics {
	/// \name Preview matrices
	/// \{
	LinearDynamicsMatrices pos, vel, acc, jerk;
	LinearDynamicsMatrices cop;		//\f$ \z = x - \frac{h}{g} \ddot x \f$
	LinearDynamicsMatrices cp;		//\f$ \xi = x + \frac{1}{\omega}\dot x \f$
	/// \}

	LinearDynamicsMatrices cont_ss, discr_ss;	//State space dynamics

	std::vector<CommonMatrixType> d_state_mat_vec;			//Vector of discrete state matrices
	std::vector<CommonMatrixType> d_state_mat_pow_vec;		//Vector of multiplied discrete state matrices
	std::vector<CommonMatrixType> rev_matrix_prod_vec;	//Products of state matrices starting from last sample
	std::vector<CommonMatrixType> d_state_mat_pow2_vec;		//Vector of multiplied discrete state matrices
	std::vector<CommonMatrixType> d_input_mat_vec;			//Vector of discrete input matrices

	void SetZero(int state_dim,
			int input_dim,
			int output_dim,
			int num_samples,
			int stable_dim,			//Number of stable modes
			int unstable_dim		//Number of unstable modes
	);
};

struct SelectionMatrices{
	CommonMatrixType sample_step, sample_step_trans;
	CommonVectorType sample_step_cx, sample_step_cy;
	CommonMatrixType Vf;
	CommonVectorType VcfX, VcfY;              			//\brief

	void SetZero();
	SelectionMatrices(int num_rows);
};

struct RelativeInequalities{//TODO: Obsolete
	CommonMatrixType x_mat, y_mat;
	CommonVectorType c_vec;

	void Resize(int rows, int cols);
};
}

#endif // MPC_WALKGEN_TYPES_H
