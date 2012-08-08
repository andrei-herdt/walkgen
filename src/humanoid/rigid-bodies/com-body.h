#pragma once
#ifndef MPC_WALKGEN_COM_BODY_H
#define MPC_WALKGEN_COM_BODY_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	com-body.h
///\brief	A class to store CoM rigid body
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author      Keith François
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////

#include "../types.h"
#include "../rigid-body.h"
#include <Eigen/Dense>

namespace MPCWalkgen{
    class CoMBody:public RigidBody{
    public:
      CoMBody(const MPCData * generalData,
              const RobotData * robotData,
              const Interpolation * interpolation);
      virtual ~CoMBody();

      virtual void interpolate(MPCSolution & solution, double currentTime, const Reference & velRef);

    protected:
      virtual void computeDynamicsMatrices(LinearDynamics & dyn,
                                           double S, double T, int N, DynamicMatrixType type);

    private:
      void interpolateTrunkOrientation(MPCSolution & solution,
                                       double /*currentTime*/, const Reference & velRef);


    };
}


#endif // MPC_WALKGEN_COM_BODY_H
