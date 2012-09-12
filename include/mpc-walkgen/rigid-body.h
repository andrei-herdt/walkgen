#pragma once
#ifndef MPC_WALKGEN_RIGID_BODY_H
#define MPC_WALKGEN_RIGID_BODY_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	rigid-body.h
///\author	Herdt Andrei
///
////////////////////////////////////////////////////////////////////////////////


#include <mpc-walkgen/types.h>
#include <mpc-walkgen/interpolation.h>
#include <vector>

namespace MPCWalkgen{
  class RigidBody{

    //
    // Public methods:
    //
  public:
    RigidBody();
    virtual ~RigidBody();

    void Init(const MPCData *mpc_parameters);
    void Init(const RobotData *data_robot);

    void ComputeDynamics();

    virtual void Interpolate(MPCSolution &solution, double currentTime, const Reference &velRef)=0;

    /// \name Accessors and mutators 
    /// \{
    inline const BodyState &state() const{return state_;}
    inline BodyState &state(){return state_;}

    inline void state(const BodyState &s){state_=s;}

    inline const LinearDynamics &dynamics_qp() const { return dynamics_qp_vec_[dynamics_iter_];};
    inline const LinearDynamics &dynamics_act() const {return dynamics_act_;};

    void setSelectionNumber(double firstSamplingPeriod);
    /// \}

  protected:
    virtual void ComputeDynamicsMatrices(LinearDynamicsMatrices &dyn,
      double sample_period_first, double sample_period_rest, int nbsamples, Derivative type) = 0;


  protected:
    const MPCData *mpc_parameters_;
    const RobotData *robot_data_p_;//TODO: Maybe should not be here
    Interpolation interpolation_;

    BodyState state_;
    /// \brief Dynamics sampled with the QP sampling rate
    std::vector<LinearDynamics> dynamics_qp_vec_;
    /// \brief Pointer to the currently relevant dynamics
    int dynamics_iter_;
    /// \brief Dynamics sampled with the actuator sampling rate
    LinearDynamics dynamics_act_;

  };
}

#endif // MPC_WALKGEN_RIGID_BODY_H
