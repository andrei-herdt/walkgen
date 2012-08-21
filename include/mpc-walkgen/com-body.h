#pragma once
#ifndef MPC_WALKGEN_COM_BODY_H
#define MPC_WALKGEN_COM_BODY_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	com-body.h
///\author	Herdt Andrei
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/types.h>
#include <mpc-walkgen/rigid-body.h>

#include <Eigen/Dense>

namespace MPCWalkgen{

  class CoMBody:public RigidBody{
  public:
    CoMBody(const MPCData *data_mpc, const RobotData *data_robot);
    virtual ~CoMBody();

    virtual void Interpolate(MPCSolution &solution, double currentTime, const Reference &velRef);

  protected:
    virtual void ComputeDynamicsMatrices(LinearDynamicsMatrices &dyn,
      double sample_period_first, double sample_period_rest, int N, Derivative type);

  private:
    void interpolateTrunkOrientation(MPCSolution &solution,
      double /*currentTime*/, const Reference &velRef);

  };
}
#endif // MPC_WALKGEN_COM_BODY_H