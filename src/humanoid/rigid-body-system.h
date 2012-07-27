#pragma once
#ifndef MPC_WALKGEN_HUMANOID_RIGID_BODY_SYSTEM_H
#define MPC_WALKGEN_HUMANOID_RIGID_BODY_SYSTEM_H

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
  namespace Humanoid{
    class RigidBodySystem{

    public:
      RigidBodySystem(const MPCData *generalData);
      ~RigidBodySystem();

      void Init(const RobotData &robot_data, const Interpolation *interpolation);

      void computeDynamics();

      void interpolateBodies(MPCSolution &solution, double currentTime, const Reference &velRef);

      void updateBodyState(const MPCSolution &solution);

      void setSelectionNumber(double firstSamplingPeriod);

      inline ConvexHull convexHull(HullType type, const SupportState &prwSupport, bool computeLinearSystem=true, bool rotateHull=true) const
      {
        ConvexHull hull;
        convexHull(hull, type, prwSupport, computeLinearSystem, rotateHull);
        return hull;
      }
      void convexHull(ConvexHull &hull, HullType type, const SupportState &prwSupport, bool computeLinearSystem=true, bool rotateHull=true) const;//TODO: Change this

      RigidBody *body(BodyType type);
      const RigidBody *body(BodyType type) const;

      inline SupportState &currentSupport() {
        return currentSupport_;
      };
      inline const SupportState &currentSupport() const {
        return currentSupport_;
      };
      inline void currentSupport(const SupportState &currentSupport) {
        currentSupport_ = currentSupport;
      };
      inline RobotData &robot_data() {
        return robot_data_;
      };

    private:
      const MPCData *generalData_;
      RobotData robot_data_;

      RigidBody *CoM_;
      RigidBody *leftFoot_;
      RigidBody *rightFoot_;

      SupportState currentSupport_;

    };

  }
}

/*! \fn MPCWalkgen::RigidBodySystem::RigidBodySystem(const MPCData *generalData
, const Interpolation *interpolation)
* \brief Constructor
*/

/*! \fn MPCWalkgen::RigidBodySystem::computeDynamics()
* \brief Compute dynamic matrices of the rigid body system
*/

/*! \fn void MPCWalkgen::RigidBodySystem::updateBodyState(const MPCSolution & solution)
* \brief Update state of bodies
* \param solution [out] the solution
*/

/*! \fn void MPCWalkgen::RigidBodySystem::firstIterationDuration(double firstIterationDuration)
* \brief Setter to define the duration of the first iteration in the QP horizon
*/

/*! \fn DynamicMatrix & MPCWalkgen::RigidBodySystem::dynamic(DynamicMatrixType type)
* \brief Return dynamic matrices of desired type
*/

/*! \fn BodyState & MPCWalkgen::RigidBodySystem::state(BodyType type)
* \brief Return state of desired body
*/

/*! \fn ConvexHull MPCWalkgen::RigidBodySystem::convexHull(HullType type,
* const SupportState & prwSupport, bool computeLinearSystem=true, bool rotateHull=true) const
* \brief Return convex hull data of hull type
* \param prwSupport previewed support associated with the convexhull
* \param computeLinearSystem if false, linear inequalities will not be computed
*/

#endif // MPC_WALKGEN_HUMANOID_RIGID_BODY_SYSTEM_H
