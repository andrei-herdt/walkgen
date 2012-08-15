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

#include <mpc-walkgen/types.h>
#include <mpc-walkgen/rigid-body.h>

#include <Eigen/Dense>

namespace MPCWalkgen{
    class FootBody:public RigidBody{
    public:
      FootBody(const MPCData * data_mpc,
               const RobotData * data_robot, Foot type);
      virtual ~FootBody();

      virtual void Interpolate(MPCSolution &solution, double currentTime, const Reference &velRef);

    protected:
    virtual void ComputeDynamicsMatrices(LinearDynamicsMatrices &dyn,
      double sample_period_first, double sample_period_rest, int nbsamples, Derivative type);

    private:
      Eigen::VectorXd &getFootVector(MPCSolution &solution, Axis axis, unsigned derivative);

      void InterpolatePolynomial(MPCSolution &solution, Axis axis, int nbSampling, double T, 
                                                const Eigen::Vector3d &state_vec,
                                                const Eigen::Vector3d &nextSupportFootState);

    private:
      Foot footType_;
    };
}

#endif // MPC_WALKGEN_FOOT_BODY_H
