#pragma once
#ifndef MPC_WALKGEN_WALKGEN
#define MPC_WALKGEN_WALKGEN

#include <mpc-walkgen/types.h>
#include <mpc-walkgen/realclock.h>
#include <mpc-walkgen/orientations-preview.h>
#include <mpc-walkgen/qp-solver.h>
#include <mpc-walkgen/qp-builder.h>
#include <mpc-walkgen/heuristic-preview.h>
#include <mpc-walkgen/rigid-body-system.h>
#include <mpc-walkgen/realclock.h>
#include <mpc-walkgen/debug.h>

namespace MPCWalkgen{

class Walkgen {

	//
	// Public methods:
	//
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW		//Necessary for vectorization of fixed-size vectors

	Walkgen();
	~Walkgen();

	void Init(MPCParameters &mpc_parameters);
	void Init(const RobotData &robot_data);

	const MPCSolution &Go(double time);
	const MPCSolution &Go();

	// \name Accessors and mutators
	// \{
	void SetPosReference(double x, double y);
	void SetVelReference(double dx, double dy, double dyaw);
	void SetVelReference(const CommonVectorType &x_vec, const CommonVectorType &y_vec, const CommonVectorType &yaw_vec);
	void SetCPReference(double global_x, double global_y, double local_x, double local_y);

	inline const Reference &cp_ref() const { return cp_ref_; }

	const BodyState &bodyState(BodyType body) const;
	void bodyState(BodyType body, const BodyState &state);
	inline const ControlOutput &output() const { return output_; };
	inline RigidBodySystem *robot() { return &robot_; };
	inline QPSolver *solver() { return solver_; };
	inline RealClock &clock() {return clock_;};
	const MPCSolution &solution() {return solution_;};
	inline const MPCParameters &mpc_parameters() const {return mpc_parameters_;};
	inline MPCParameters &mpc_parameters() {return mpc_parameters_;};
	// \}

	//
	// Private methods:
	//
private:
	void Init();
	void BuildProblem();
	void GenerateTrajectories();
	void ResetOutputIndex();
	void IncrementOutputIndex();
	//Copies the output_index_ trajectory entries to output_;
	void UpdateOutput();
	void ResetCounters(double time);
	void SetCounters(double time);

	void StoreResult();
	void SetWalkingMode();

	//
	// Private data members:
	//
private:
	MPCParameters mpc_parameters_;
	RobotData robot_data_;

	::MPCWalkgen::QPSolver *solver_;
	QPBuilder *builder_;
	HeuristicPreview *heur_preview_;
	RigidBodySystem robot_;

	OrientationsPreview *orient_preview_;

	RealClock clock_;

	// \brief Contains pointers to trajectory values
	// an internal logic set the pointers the currently relevant indeces
	ControlOutput output_;
	int output_index_;

	MPCSolution solution_;
	Reference vel_ref_, pos_ref_, cp_ref_;
	/// \brief The new value of reference velocity, updated with in online method
	Reference new_vel_ref_;

	/// \brief Time at which the problem should be updated
	double first_coarse_sample_;
	double first_fine_sample_;
	double next_computation_;
	double next_act_sample_;

	/// \brief Synchronised time with QP sampling
	double current_time_;

	double last_des_cop_x_, last_des_cop_y_;
	double last_first_contr_x_, last_first_contr_y_;
};
}



#endif // MPC_WALKGEN_WALKGEN
