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

namespace MPCWalkgen{

class Walkgen {
	//
	// Public methods:
	//
public:
	Walkgen();
	~Walkgen();

	void Init(const MPCParameters &mpc_parameters);
	void Init(const RobotData &robot_data);

	const MPCSolution &Go(double time);
	const MPCSolution &Go();

	// \name Accessors and mutators
	// \{
	void SetReference(double dx, double dy, double dyaw);
	void SetReference(CommonVectorType dx, CommonVectorType dy, CommonVectorType dyaw);

	const BodyState &bodyState(BodyType body) const;
	void bodyState(BodyType body, const BodyState &state);
	inline const ControlOutput &output() const { return output_; };
	inline RigidBodySystem *robot() { return robot_; };

	inline const QPSolver *solver() const { return solver_; };

	RealClock &clock() {return clock_;};

	const MPCSolution &solution() {return solution_;};
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
	void UpdateOutput();
	void ResetCounters(double time);
	void SetCounters(double time);

	//
	// Private data members:
	//
private:
	MPCParameters mpc_parameters_;
	RobotData robot_data_;

	::MPCWalkgen::QPSolver *solver_;
	QPBuilder *builder_;
	HeuristicPreview *preview_;
	RigidBodySystem *robot_;

	OrientationsPreview *orient_preview_;

	RealClock clock_;

	// \brief Contains pointers to trajectory values
	// an internal logic set the pointers the currently relevant indeces
	ControlOutput output_;
	int output_index_;

	MPCSolution solution_;
	Reference vel_ref_, pos_ref_;
	/// \brief The new value of reference velocity, updated with in online method
	Reference new_vel_ref_;
	WeightCoefficients weight_coefficients_;

	/// \brief Time at which the problem should be updated
	double first_sample_time_;
	double next_computation_;
	double next_act_sample_;

	/// \brief Synchronised time with QP sampling
	double current_time_;
};
}



#endif // MPC_WALKGEN_WALKGEN
