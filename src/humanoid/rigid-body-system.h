#pragma once
#ifndef MPC_WALKGEN_RIGID_BODY_SYSTEM_H
#define MPC_WALKGEN_RIGID_BODY_SYSTEM_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	rigid-body-system.h
///\brief	A class to store the rigid body model
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author      Keith François
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////


#include "types.h"
#include "rigid-body.h"

#include <Eigen/Dense>
#include <vector>

namespace MPCWalkgen{
    class RigidBodySystem{

    public:
      RigidBodySystem(const MPCData *generalData);
      ~RigidBodySystem();

      void Init(const RobotData &data_robot, const Interpolation *interpolation);

      void interpolateBodies(MPCSolution &solution, double currentTime, const Reference &velRef);

      void UpdateState(const MPCSolution &solution);

      void setSelectionNumber(double firstSamplingPeriod);

    private:
      void ComputeDynamics();

    private:
      const MPCData *data_mpc_;
      RobotData data_robot_;

      RigidBody *com_;
      RigidBody *foot_left_;
      RigidBody *foot_right_;

      SupportState currentSupport_;

    #include "rigid-body-system-inl.h"
    };
}
#endif // MPC_WALKGEN_RIGID_BODY_SYSTEM_H
