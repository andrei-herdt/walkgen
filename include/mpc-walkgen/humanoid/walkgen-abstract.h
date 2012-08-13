////////////////////////////////////////////////////////////////////////////////
///
///\file	walkgen-abstract.h
///\brief	Abstract class to instanciate Walkgen algorithm for humanoid robot
///\author	Herdt Andrei
///\author      Keith François
///\author	Lafaye Jory
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////


#pragma once
#ifndef MPC_WALKGEN_WALKGEN_ABSTRACT_H
#define MPC_WALKGEN_WALKGEN_ABSTRACT_H


//TODO: Change this ... interface
#include <mpc-walkgen/humanoid/sharedpgtypes.h>

#include <Eigen/Core>



namespace MPCWalkgen{

    class RigidBodySystem;

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
      /// \param[in] mpcData: data relative to the qp solver
      virtual void Init(const RobotData &data_robot, const MPCData &mpcData) = 0;
      virtual void Init() = 0;

      /// \brief Call method to handle on-line generation of ZMP reference trajectory.
      /// \param[in] time : Current time.
      /// \param[in] previewBodiesNextState
      /// \return The associated solution
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

      virtual const BodyState &bodyState(BodyType body)const = 0;
      virtual void bodyState(BodyType body, const BodyState &state) = 0;

      virtual const ControlOutput &output() = 0;

      virtual RigidBodySystem *robot() = 0;
      /// \}

    };
    /*! Factory of Pattern generator interface. */
    MPC_WALKGEN_API WalkgenAbstract * createWalkgen();
}


#endif // MPC_WALKGEN_WALKGEN_ABSTRACT_H
