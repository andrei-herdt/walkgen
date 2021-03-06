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
			RealClock *clock,
			double *last_des_cop_x,//TODO: Temporary solutions
			double *last_des_cop_y,
			double *last_first_contr_x,
			double *last_first_contr_y
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

	void BuildEqualityConstraints(const MPCSolution &solution);

	void BuildInequalityConstraints(const MPCSolution &solution);

	void BuildCPEqConstraints(const MPCSolution &solution);

	void BuildFootPosEqConstraints(const MPCSolution &solution);

	void BuildFootPosEqConstraintsSag(const MPCSolution &solution);

	void BuildCoPEqConstraints(const MPCSolution &solution);

	void BuildFootPosIneqConstraints(const MPCSolution &solution);

	void BuildFootVelIneqConstraints(const MPCSolution &solution);

	void BuildCoPIneqConstraints(const MPCSolution &solution);

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

	CommonVectorType tmp_vec_, tmp_vec2_,tmp_vec3_, tmp_vec4_;
	CommonMatrixType tmp_mat_, tmp_mat2_;

	CommonMatrixType contr_mov_mat_, contr_mov_mat_tr_;

	double current_time_;

	RealClock *clock_;

	double *last_des_cop_x_, *last_des_cop_y_;
	double *last_first_contr_x_, *last_first_contr_y_;

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

	std::vector<CommonMatrixType> u_trans_v_mat_vec_;
	std::vector<CommonMatrixType> v_trans_v_mat_vec_;
	std::vector<CommonMatrixType> v_trans_vc_mat_vec_;
	std::vector<CommonMatrixType> v_trans_s_mat_vec_;
	std::vector<CommonMatrixType> u_trans_vc_mat_vec_;
	std::vector<CommonMatrixType> u_trans_s_mat_vec_;
	std::vector<CommonMatrixType> u_trans_ref_mat_vec_;
	std::vector<CommonMatrixType> v_trans_ref_mat_vec_;


	std::vector<CommonMatrixType> cop_pen_mat_vec_;
	std::vector<CommonMatrixType> dcop_mat_vec_;
	std::vector<CommonMatrixType> dcop_mat_tr_vec_;
	std::vector<CommonMatrixType> dcop_pen_mat_vec_;

	std::vector<CommonMatrixType> cp_fp_pen_mat_vec_;
	std::vector<CommonMatrixType> vel_pen_mat_vec_;

	ConvexHull foot_hull_edges_;
	ConvexHull cop_hull_edges_;
	ConvexHull hull_;

	CommonMatrixType state_trans_mat_;


};
}
#endif // MPC_WALKGEN_QP_BUILDER_H
