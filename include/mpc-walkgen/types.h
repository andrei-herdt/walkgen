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

//
// Enums:
//
enum HullType{ FOOT_HULL, COP_HULL };//TODO: Remove this

enum QPMatrixType{ HESSIAN, matrixA };//TODO: Remove this

enum QPVectorType{ vectorP,  vectorBU,  vectorBL,  vectorXU,  vectorXL };//TODO: Remove this

enum Derivative {
	POSITION		= 0,
	VELOCITY		= 1,
	ACCELERATION	= 2,
	JERK			= 3,
	COP			= 4
};

enum SampleRate { QP, ACTUATORS };//TODO: Needed?

enum Phase { SS, DS };

enum Foot { LEFT, RIGHT };

enum BodyType { LEFT_FOOT, RIGHT_FOOT, COM  };

enum SolverName { QPOASES, LSSOL };

enum Axis { X, Y, Z, Yaw };

enum DynamicsOrder { FIRST_ORDER = 1, SECOND_ORDER = 2, THIRD_ORDER = 3 };

enum Mode { INITIAL = 0, STANDING = 1, WALK = 2 };

//
// Typedefs:
//
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> EigenMatrixXdRM;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> EigenMatrixXdCM;
typedef Eigen::Matrix2d Matrix2D;
typedef EigenMatrixXdRM CommonMatrixType;

typedef Eigen::Vector2d Vector2D;
typedef Eigen::VectorXd CommonVectorType;

//
// Data structures:
//
struct Frame{
	CommonVectorType x;
	CommonVectorType y;
	CommonVectorType yaw;

	Frame();

	void resize(int size);
};

struct Reference{
	Frame global;
	Frame local;

	Reference();

	void resize(int size);
};

struct MPC_WALKGEN_API BodyState{
	Eigen::Vector3d x;
	Eigen::Vector3d y;
	Eigen::Vector3d z;
	Eigen::Vector3d yaw;
	Eigen::Vector3d pitch;
	Eigen::Vector3d roll;

	BodyState();

	void reset();
};

struct MPC_WALKGEN_API FootData{
	double soleWidth;
	double soleHeight;

	Eigen::Vector3d anklePositionInLocalFrame;

	std::vector<double> edges_x_vec;
	std::vector<double> edges_y_vec;

	std::vector<double> position;

	FootData();
	FootData(const FootData &f);//TODO: LocalAnklePosition_ better?
			~FootData();

			void SetEdges(double front, double back, double left, double right, double security_margin);
};

struct MPC_WALKGEN_API HipYawData {
	double lowerBound;
	double upperBound;
	double lowerVelocityBound;
	double upperVelocityBound;
	double lowerAccelerationBound;
	double upperAccelerationBound;

	HipYawData();
	~HipYawData();
};

struct MPC_WALKGEN_API WeightCoefficients{
	std::vector<double> pos;
	std::vector<double> vel;
	std::vector<double> acc;
	std::vector<double> jerk;
	std::vector<double> cop;
	std::vector<double> control;

	//std::map<Mode, int> mode;

	/// \brief Define the element of weight_coefficients std::vector used in this iteration
	int active_mode;
	bool is_initial_mode;

	void SetCoefficients(Reference &ref);

	WeightCoefficients(int num_modes = 2);
	~WeightCoefficients();
};

struct MPC_WALKGEN_API SupportState {
	Phase phase;
	Foot foot;

	int nbStepsLeft;
	int step_number;
	int nbInstants;

	double time_limit;
	double start_time;

	double x, y, yaw;
	double yawTrunk;//TODO: Why in SupportState? -> for compatibility with temporary previewROrientation class

	bool state_changed;

	/// \brief Define if the support state is in a (transitional) double support phase
	bool transitional_ds;

	/// \brief The length of the previous sampling period (can be different from period_qpsample)
	double previousSamplingPeriod;//TODO: change name

	// \brief The relative weight of this support state in the QP (A support state duration of QPSamplingTime have : iterationWeight = 1)
	double sampleWeight;//TODO: shouldn't it be outside... somewhere...?
};

struct MPC_WALKGEN_API ConvexHull {
	/// \brief Set of vertices
	CommonVectorType x;
	CommonVectorType y;
	CommonVectorType z;

	/// \brief Set of inequalities A*x + B*y + C*z + D>0
	CommonVectorType A;
	CommonVectorType B;
	CommonVectorType C;
	CommonVectorType D;

	ConvexHull();
	~ConvexHull();

	ConvexHull &operator= (const ConvexHull &hull); // TODO: CopyFrom() instead of =
	void resize(int size);
	void rotate(double yaw);
	void computeLinearSystem(const Foot &foot);
};

struct SolverData {
	SolverName name;
	int num_wsrec;  	//Maximal number of working set recomputations
	bool analysis;
};

struct MPC_WALKGEN_API MPCParameters {
	double period_qpsample;		//Sampling period [sec]
	double period_mpcsample;	//Time between recomputations [sec]
	double period_actsample;	//Actuator sampling period [sec]

	int num_samples_horizon;  	//Number of samplings inside horizon

	int nbqpsamples_step;		//Step period ss_left<->ss_right in qp sample periods
	int nbqpsamples_dsss;		//Length of initial double support phase [num. samples]
	int nbsteps_ssds;			//Steps before halt

	double period_ds;			//Length of the (permanent) double support phase (should be a large value)

	bool warmstart;

	bool interpolate_whole_horizon;			//Interpolate not only the control (first element) but for the whole preview period

	bool closed_loop;

	DynamicsOrder dynamics_order;

	WeightCoefficients weight_coefficients;

	SolverData solver;

	/// \brief Compute the number of recomputations left until next sample
	int nbFeedbackSamplesLeft(double firstSamplingPeriod) const;
	/// \brief Number of simulation iterations between two feedback call
	int num_samples_act() const;

	int nbFeedbackSamplesStandard() const;			/// \brief Number of feedback iterations between two QP instants

	int num_qpsamples_ss() const;
	int num_steps_max() const;

	double period_ss() const;
	double period_trans_ds() const;

	MPCParameters();
	~MPCParameters();
};

struct MPC_WALKGEN_API RobotData {
	double mass;
	double max_foot_height;
	double max_foot_vel;
	double security_margin;

	Eigen::Vector3d com;

	FootData leftFoot;
	FootData rightFoot;

	HipYawData left_hip_yaw;
	HipYawData right_hip_yaw;

	ConvexHull left_foot_pos_hull;
	ConvexHull right_foot_pos_hull;
	ConvexHull left_foot_ss_hull;
	ConvexHull right_foot_ss_hull;
	ConvexHull left_foot_ds_hull;
	ConvexHull right_foot_ds_hull;

	RobotData(const FootData &leftFoot, const FootData &rightFoot,
			const HipYawData &leftHipYaw, const HipYawData &rightHipYaw,
			double mass);
	RobotData();
	~RobotData();

	void SetCoPHulls(double ds_distance);
	//TODO: void ComputeMaxFootVel();
};

struct MPC_WALKGEN_API Trajectory {
	CommonVectorType x_vec, y_vec, z_vec, yaw_vec;
};

struct Motion {
	Trajectory pos, vel, acc, jerk;
	Trajectory control;

	void resize(int size);
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
	CommonVectorType initialSolution;

	Eigen::VectorXi constraints;
	Eigen::VectorXi initialConstraints;

	/// \brief True if a new trajectory is computed in online loop
	bool newTraj;//TODO: Remove this

	std::vector<double> sampling_times_vec;	/// starting with 0, i.e. all times are relative to the current time

	std::vector<SupportState> support_states_vec;

	std::vector<double> support_yaw_vec;
	std::vector<double> trunk_yaw_vec;

	CommonVectorType CoPTrajX;
	CommonVectorType CoPTrajY;

	Motion com_prw, cop_prw, left_foot_prw, right_foot_prw;
	Motion com_act, cop_act, left_foot_act, right_foot_act;

	SolutionAnalysis analysis;

	Reference pos_ref;

	struct State {
		CommonVectorType CoMTrajX_;
		CommonVectorType CoMTrajY_;
		//TODO: Add CoMTrajZ_;

		CommonVectorType leftFootTrajX_;
		CommonVectorType leftFootTrajY_;
		CommonVectorType leftFootTrajZ_;
		CommonVectorType leftFootTrajYaw_;

		CommonVectorType rightFootTrajX_;
		CommonVectorType rightFootTrajY_;
		CommonVectorType rightFootTrajZ_;
		CommonVectorType rightFootTrajYaw_;

		CommonVectorType trunkYaw_;
	};
	std::vector<State> state_vec;

	MPCSolution& operator = (MPCSolution const &);

	void reset();

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
	CommonMatrixType state_mat;
	CommonMatrixType input_mat;
	CommonMatrixType input_mat_tr;
	CommonMatrixType input_mat_inv;
	CommonMatrixType input_mat_inv_tr;

	void SetZero(double state_dimension, double num_samples);
};

struct LinearDynamics {
	LinearDynamicsMatrices pos, vel, acc, jerk, cop;

	void SetZero(double state_dimension, double input_dimension);
};

struct SelectionMatrices{
	CommonMatrixType sample_step, sample_step_trans;
	CommonVectorType sample_step_cx, sample_step_cy;
	CommonMatrixType Vf;
	CommonVectorType VcfX, VcfY;              			//\brief
	CommonVectorType sample_mstep_cx, sample_mstep_cy;  //\brief Middle of both currently supporting feet
	CommonMatrixType sample_mstep, sample_mstep_trans;	//\brief Middle of previewed feet

	void SetZero();
	SelectionMatrices(int num_rows);
};

struct RelativeInequalities{//TODO: Obsolete
	CommonMatrixType DX;
	CommonMatrixType DY;
	CommonVectorType Dc;

	void resize(int rows, int cols);
};
}

#endif // MPC_WALKGEN_TYPES_H
