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

      void Init(const RobotData &robot_data);

      void Interpolate(MPCSolution &solution, double currentTime, const Reference &velRef);

      void UpdateState(const MPCSolution &solution);

      void setSelectionNumber(double firstSamplingPeriod);

      void ComputeDynamics();


      //
      // Private data members:
      //
    private:
      const MPCData *mpc_parameters_p_;
      RobotData robot_data_;

      DynamicsBuilder dynamics_builder_;

      RigidBody *com_;
      RigidBody *left_foot_;
      RigidBody *right_foot_;

      SupportState currentSupport_;

    #include <mpc-walkgen/rigid-body-system-inl.h>
    };
}
#endif // MPC_WALKGEN_RIGID_BODY_SYSTEM_H
