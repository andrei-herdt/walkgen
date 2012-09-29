#pragma once
#ifndef MPC_WALKGEN_DYNAMICS_BUILDER_H
#define MPC_WALKGEN_DYNAMICS_BUILDER_H

#include <mpc-walkgen/types.h>

namespace MPCWalkgen{
  class DynamicsBuilder{
 
    //
    // Public methods:
    //
  public:
    DynamicsBuilder();

    ~DynamicsBuilder();

    void Init();

    void Build(DynamicsOrder dynamics_order,
      LinearDynamics &dyn,
      double height,
      double sample_period_first, 
      double sample_period_rest, 
      int nbsamples);

    //
    // Private methods:
    //
  private:

    void BuildSecondOrder(LinearDynamics &dyn,  double height, double sample_period_first, double sample_period_rest, int num_samples);
    void BuildThirdOrder(LinearDynamics &dyn,  double height, double sample_period_first, double sample_period_rest, int num_samples);

    void BuildSecondOrder(LinearDynamicsMatrices &dyn,
      double height,
      double sample_period_first, 
      double sample_period_rest, 
      int nbsamples, 
      Derivative derivative);

    void BuildSecondOrderCoP(LinearDynamics &dyn,
      double height,
      double sample_period_first,
      double sample_period_rest,
      int nbsamples,
      Derivative derivative);
 
    void BuildThirdOrder(LinearDynamicsMatrices &dyn,
      double height,
      double sample_period_first, 
      double sample_period_rest, 
      int nbsamples, 
      Derivative derivative);

    void ComputeDiscreteStateMat(Matrix2D &mat, double sample_period_first);
    void ComputeDiscreteInputMat(Matrix2D &mat, Matrix2D &discr_state_mat);

    //
    // Private data members:
    //
  private:
    Matrix2D cont_state_mat_, cont_state_mat_inv_;
    Vector2D eigenval_vec_;
    Matrix2D eigenvec_mat_, eigenvec_mat_inv_;
    Matrix2D cont_input_vec_;

    Matrix2D diag_exp_eig_mat_;	//\f[ e^{\lambda_1 T} \f]

    Matrix2D identity_mat_;
    Matrix2D precomp_input_mat_;  // \f[ -A^{-1}\mathbb{I}B

    Matrix2D tmp_mat_;

  };
}
#endif // MPC_WALKGEN_DYNAMICS_BUILDER_H
