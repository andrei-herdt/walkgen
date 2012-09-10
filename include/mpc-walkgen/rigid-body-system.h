#pragma once
#ifndef MPC_WALKGEN_RIGID_BODY_SYSTEM_H
#define MPC_WALKGEN_RIGID_BODY_SYSTEM_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	rigid-body-system.h
///\brief	A class to store the rigid body model
///\author	Herdt Andrei
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/types.h>
#include <mpc-walkgen/rigid-body.h>

#include <Eigen/Dense>
#include <vector>

namespace MPCWalkgen{
    class RigidBodySystem{

      // 
      // Public methods:
      //
    public:
      RigidBodySystem();
      ~RigidBodySystem();

      void Init(const MPCData *mpc_parameters_p);

      void Init(const RobotData &data_robot, const Interpolation *interpolation);

      void interpolateBodies(MPCSolution &solution, double currentTime, const Reference &velRef);

      void UpdateState(const MPCSolution &solution);

      void setSelectionNumber(double firstSamplingPeriod);

    private:
      void ComputeDynamics();

    private:
      const MPCData *mpc_parameters_p_;
      RobotData data_robot_;

      RigidBody *com_;
      RigidBody *foot_left_;
      RigidBody *foot_right_;

      SupportState currentSupport_;

    #include <mpc-walkgen/rigid-body-system-inl.h>
    };
}
#endif // MPC_WALKGEN_RIGID_BODY_SYSTEM_H
