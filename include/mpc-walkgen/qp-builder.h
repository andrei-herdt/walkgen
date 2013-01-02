#pragma once
#ifndef MPC_WALKGEN_QP_BUILDER_H
#define MPC_WALKGEN_QP_BUILDER_H

#include <mpc-walkgen/types.h>
#include <mpc-walkgen/qp-solver.h>
#include <mpc-walkgen/heuristic-preview.h>
#include <mpc-walkgen/realclock.h>

namespace MPCWalkgen{
class QPBuilder{

	//
	// Public methods:
	//
public:
	QPBuilder(HeuristicPreview *preview,
			QPSolver *solver,
			Reference *pos_ref,
			Reference *vel_ref,
			Reference *cp_ref,
			RigidBodySystem *robot,
			MPCParameters *mpc_parameters,
			RealClock *clock
			);
	~QPBuilder();

	void PrecomputeObjective();

	void BuildProblem(MPCSolution &solution);

	void TransformControlVector(MPCSolution &solution);

	void BuildGlobalVelocityReference(const MPCSolution &solution);

	inline void current_time(double time) {current_time_ = time;};

	//
	// Private methods:
	//
private:

	void BuildFootPosInequalities(const MPCSolution &solution);

	void BuildObjective(const MPCSolution &solution);

	void BuildInequalityConstraints(const MPCSolution &solution);

	void BuildCPConstraints(const MPCSolution &solution);

	void BuildFootPosConstraints(const MPCSolution &solution);

	void BuildFootVelConstraints(const MPCSolution &solution);

	void BuildCoPConstraints(const MPCSolution &solution);

	void BuildTerminalConstraints(const MPCSolution &solution);

	void ComputeWarmStart(MPCSolution &solution);

	//
	// Private data members:
	//
private:

	HeuristicPreview *preview_;
	QPSolver *solver_;
	RigidBodySystem *robot_;

	Reference *pos_ref_, *vel_ref_, *cp_ref_;

	MPCParameters *mpc_parameters_;

	CommonVectorType tmp_vec_, tmp_vec2_;
	CommonMatrixType tmp_mat_, tmp_mat2_;

	double current_time_;

	RealClock *clock_;

	RelativeInequalities foot_inequalities_;

	std::vector<CommonMatrixType> curr_cop_variant_;

	std::vector<CommonMatrixType> const_hessian_mat_;
	std::vector<CommonMatrixType> const_hessian_n_mat_;
	std::vector<CommonMatrixType> const_cholesky_;

	std::vector<CommonMatrixType> state_variant_;   // These elements are multiplied by the state
	std::vector<CommonMatrixType> select_variant_;

	std::vector<CommonMatrixType> ref_variant_vel_;
	std::vector<CommonMatrixType> ref_variant_pos_;
	std::vector<CommonMatrixType> ref_variant_cp_;

	ConvexHull foot_hull_edges_;
	ConvexHull cop_hull_edges_;
	ConvexHull hull_;


};
}
#endif // MPC_WALKGEN_QP_BUILDER_H
