#pragma once
#ifndef MPC_WALKGEN_FOOT_BODY_H
#define MPC_WALKGEN_FOOT_BODY_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	com-body.h
///\brief	A class to store CoM rigid body
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author      Keith François
////////////////////////////////////////////////////////////////////////////////

#include "../types.h"
#include "../rigid-body.h"

#include <Eigen/Dense>

namespace MPCWalkgen{
    class FootBody:public RigidBody{
    public:
      FootBody(const MPCData * generalData,
               const RobotData * robotData, Foot type);
      virtual ~FootBody();

      virtual void Interpolate(MPCSolution &solution, double currentTime, const Reference &velRef);

    protected:
    virtual void ComputeDynamicsMatrices(LinearDynamicsMatrices &dyn,
      double sample_period_first, double sample_period_rest, int nbsamples, Derivative type);

    private:
      Eigen::VectorXd &getFootVector(MPCSolution &solution, Axis axis, unsigned derivative);

      void computeFootInterpolationByPolynomial(MPCSolution &solution, Axis axis, int nbSampling,
                                                const Eigen::Vector3d &FootCurrentState,
                                                double T, const Eigen::Vector3d &nextSupportFootState);

    private:
      Foot footType_;
    };
}

#endif // MPC_WALKGEN_FOOT_BODY_H
