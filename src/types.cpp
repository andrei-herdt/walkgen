#include <mpc-walkgen/types.h>

using namespace MPCWalkgen;

void LinearDynamicsMatrices::SetZero(double state_dimension, double num_samples) {
    state_mat.setZero(num_samples, state_dimension);
    input_mat.setZero(num_samples, num_samples);
    input_mat_tr.setZero(num_samples, num_samples);
    input_mat_inv.setZero(num_samples, num_samples);
    input_mat_inv_tr.setZero(num_samples, num_samples);
}

void LinearDynamics::SetZero(double state_dimension, double num_samples) {
	pos.SetZero(state_dimension, num_samples);
	vel.SetZero(state_dimension, num_samples);
	acc.SetZero(state_dimension, num_samples);
	jerk.SetZero(state_dimension, num_samples);
	cop.SetZero(state_dimension, num_samples);
}


SelectionMatrices::SelectionMatrices(int num_rows)
:sample_step(num_rows, num_rows)
,sample_step_trans(num_rows, num_rows)
,sample_step_cx(num_rows)
,sample_step_cy(num_rows)
,Vf(num_rows, num_rows)
,VcfX(num_rows)
,VcfY(num_rows)
,sample_mstep_cx(num_rows)
,sample_mstep_cy(num_rows)
,sample_mstep(num_rows, num_rows)
,sample_mstep_trans(num_rows, num_rows)
{}

void SelectionMatrices::SetZero() {
	sample_step.setZero(); sample_step_trans.setZero();
	sample_step_cx.setZero(); sample_step_cy.setZero();
	Vf.setZero();
	VcfX.setZero();  VcfY.setZero();
	sample_mstep_cx.setZero(); sample_mstep_cy.setZero();
	sample_mstep.setZero(); sample_mstep_trans.setZero();
}

void RelativeInequalities::resize(int rows, int cols){
	if (rows!=DX.rows() || cols!=DX.cols()){
		DX.setZero(rows, cols);
		DY.setZero(rows, cols);
		Dc.setZero(rows, cols);
	}else{
		DX.fill(0);
		DY.fill(0);
		Dc.fill(0);
	}

}
