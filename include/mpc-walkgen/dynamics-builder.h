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
 
    void BuildThirdOrder(LinearDynamicsMatrices &dyn,
      double height,
      double sample_period_first, 
      double sample_period_rest, 
      int nbsamples, 
      Derivative derivative);

  };
}
#endif // MPC_WALKGEN_DYNAMICS_BUILDER_H