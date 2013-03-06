#pragma once
#ifndef MPC_WALKGEN_HEURISTIC_PREVIEW_H
#define MPC_WALKGEN_HEURISTIC_PREVIEW_H

#include <mpc-walkgen/rigid-body-system.h>
#include <mpc-walkgen/state-fsm.h>
#include <mpc-walkgen/types.h>
#include <mpc-walkgen/realclock.h>

namespace MPCWalkgen{
class HeuristicPreview{

	//
	// Public methods:
	//
public:
	HeuristicPreview(Reference *ref,
			RigidBodySystem *robot,
			const MPCParameters *mpc_parameters,
			RealClock *clock
	);
	~HeuristicPreview();

	void PreviewSamplingTimes(double current_time,
			double first_fine_period,
			double first_coarse_period,
			MPCSolution &solution
	);

	void PreviewSupportStates(double first_sample_period,
			MPCSolution &solution
	);

	void BuildRotationMatrix(MPCSolution &solution
	);

	inline SelectionMatrices &selection_matrices(){return select_matrices_;}

	inline const CommonMatrixType &rot_mat() const{return rot_mat_;}
	inline const CommonMatrixType &rot_mat2() const{return rot_mat2_;}
	inline const CommonMatrixType &rot_mat2_tr() const{return rot_mat2_tr_;}

	//
	// Private methods:
	//
private:
	void BuildSelectionMatrices(MPCSolution &solution);

	//
	// Private data members:
	//
private:
	RigidBodySystem *robot_;
	const MPCParameters *mpc_parameters_;
	StateFSM *support_fsm_;

	SelectionMatrices select_matrices_;

	CommonMatrixType rot_mat_;
	CommonMatrixType rot_mat2_;
	CommonMatrixType rot_mat2_tr_;

	RealClock *clock_;

};
}

#endif // MPC_WALKGEN_HEURISTIC_PREVIEW_H
