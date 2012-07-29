#pragma once
#ifndef MPC_WALKGEN_HUMANOID_RIGID_BODY_H
#define MPC_WALKGEN_HUMANOID_RIGID_BODY_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	rigid-body.h
///\brief	A class to store rigid bodies
///\author	Lafaye Jory
///\author      Keith François
///\author	Herdt Andrei
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////


#include "types.h"
#include "../common/interpolation.h"
#include <vector>

namespace MPCWalkgen{
  namespace Humanoid{
    class RigidBody{

    public:
      RigidBody(const MPCData * generalData,
                const RobotData * robotData,
                const Interpolation * interpolation);
      virtual ~RigidBody();

      inline const BodyState & state() const{return state_;}
      inline BodyState & state(){return state_;}

      inline void state(const BodyState & s){state_=s;}

      const LinearDynamics & dynamics(DynamicMatrixType type) const;

      void setSelectionNumber(double firstSamplingPeriod);

      void computeDynamics();

      virtual void interpolate(MPCSolution & result, double currentTime, const Reference & velRef)=0;

    protected:
      virtual void computeDynamicsMatrices(LinearDynamics & dyn,
                                           double S, double T, int N, DynamicMatrixType type)=0;


    protected:
      const MPCData *generalData_;
      const RobotData *robotData_;
      const Interpolation *interpolation_;

      BodyState state_;
      std::vector<LinearDynamics> pos_dynamics_vec_;
      std::vector<LinearDynamics> vel_dynamics_vec_;
      std::vector<LinearDynamics> acc_dynamics_vec_;
      std::vector<LinearDynamics> jerk_dynamics_vec_;
      std::vector<LinearDynamics> cop_dynamics_vec_;

      LinearDynamics posInterpol_;
      LinearDynamics velInterpol_;
      LinearDynamics accInterpol_;
      LinearDynamics copInterpol_;

      int matrixNumber_;
    };
  }
}

#endif // MPC_WALKGEN_HUMANOID_RIGID_BODY_H
