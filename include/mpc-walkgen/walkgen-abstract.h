#pragma once
#ifndef MPC_WALKGEN_WALKGEN_ABSTRACT_H
#define MPC_WALKGEN_WALKGEN_ABSTRACT_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	walkgen-abstract.h
///\author	Herdt Andrei
///\author      Keith François
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/sharedtypes.h>

#include <Eigen/Core>

namespace MPCWalkgen{

    class RigidBodySystem;
    class QPSolver;
    class RealClock;

    class  WalkgenAbstract
    {
      //
      // Public methods:
      //
    public:

      WalkgenAbstract();

      virtual ~WalkgenAbstract() =0;

      /// \brief Initialize the system
      /// \param[in] data_robot: data relative to the robot
      /// \param[in] mpc_parameters: data relative to the qp solver
      virtual void Init(const RobotData &data_robot, const MPCData &mpc_parameters) = 0;
      virtual void Init() = 0;

      /// \brief Call method to handle on-line generation of ZMP reference trajectory.
      /// \param[in] time : Current time.
      /// \return Solution
      ///   If solution.newTraj is true, the method has succeeded.
      virtual const MPCSolution &online(double time) = 0;
      virtual const MPCSolution &online() = 0;

      /// \name Accessors and mutators
      /// \{
      /// \brief Set the reference (velocity only as for now)
      virtual void reference(double dx, double dy, double dyaw) = 0;
      virtual void reference(Eigen::VectorXd dx, Eigen::VectorXd dy, Eigen::VectorXd dyaw) = 0;
      /// \}

      /// \name Accessors relative to the state of the robot.
      /// \{
      virtual const SupportState &currentSupportState() const = 0;
      virtual void currentSupportState(const SupportState &newSupportState)=0;

      virtual const BodyState &bodyState(BodyType body) const = 0;
      virtual void bodyState(BodyType body, const BodyState &state) = 0;

      virtual const ControlOutput &output() const = 0;

      virtual RigidBodySystem *robot() = 0;
      virtual const QPSolver *solver() const = 0;

      virtual RealClock &clock() = 0;
	  virtual const MPCSolution &solution() = 0;
      /// \}

    };
    /// \brief Factory of Pattern generator interface. 
    MPC_WALKGEN_API WalkgenAbstract *createWalkgen();
}


#endif // MPC_WALKGEN_WALKGEN_ABSTRACT_H
