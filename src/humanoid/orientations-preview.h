
/*
 * OrientationsPreview.h
 * Temporary class
 */
#pragma once
#ifndef MPC_WALKGEN_ORIENTATIONS_PREVIEW_H
#define MPC_WALKGEN_ORIENTATIONS_PREVIEW_H



#include <deque>
#include <vector>


#include "types.h"
#include "rigid-body-system.h"

namespace MPCWalkgen
{

struct  COMState_s2
{
  double x[3],y[3],z[3];
  double yaw[3]; // aka theta
  double pitch[3]; // aka omega
  double roll[3]; // aka hip



  void reset(){
	    for(unsigned int i=0;i<3;i++)
	      {
		x[i] = 0.0; y[i] = 0.0; z[i] = 0.0;
		yaw[i] = 0.0; pitch[i] = 0.0; roll[i] = 0.0;
	      }
	  }

  COMState_s2() {
	    reset();
	  }
};

typedef struct COMState_s2 COMState2;

  /// \brief The acceleration phase is fixed
  class OrientationsPreview {

    //
    // Public methods:
    //
  public:

    OrientationsPreview( );
    ~OrientationsPreview();

    void init(const MPCData &mpcData, const RobotData &robotData);

    /// \brief Preview feet and trunk orientations inside the preview window
    /// The orientations of the feet are adapted to the previewed orientation of the hip.
    /// The resulting velocities accelerations and orientations are verified against the limits.
    /// If the constraints can not be satisfied the rotational velocity of the trunk is reduced.
    /// The trunk is rotating with a constant speed after a constant acceleration phase of T_ length.
    /// During the initial double support phase the trunk is not rotating contrary to the following.
    ///
    /// \param[in] Time
    /// \param[in] Ref
    /// \param[in] StepDuration
    /// \param[in] LeftFoot
    /// \param[in] RightFoot
    /// \param[out] Solution Trunk and Foot orientations
    void preview_orientations(double Time,
        const Reference &Ref,
        double StepDuration,
        const BodyState &LeftFoot,
        const BodyState &RightFoot,
        MPCSolution &Solution);

    /// \brief Interpolate previewed orientation of the trunk
    ///
    /// \param[in] robot
    void interpolate_trunk_orientation(const RigidBodySystem * robot);

    /// \name Accessors
    /// \{
    inline COMState2 const &CurrentTrunkState() const
    { return TrunkState_; };
    inline void CurrentTrunkState(const COMState2 &TrunkState)
    { TrunkState_ = TrunkState; };
    inline double SSLength() const
    { return SSPeriod_; };
    inline void SSLength( double SSPeriod)
    { SSPeriod_ = SSPeriod; };
    inline double SamplingPeriod() const
    { return T_; };
    inline void SamplingPeriod( double SamplingPeriod)
    { T_ = SamplingPeriod; };
    inline double SimuPeriod() const
    { return Ti_; };
    inline void SimuPeriod( double SamplingPeriod)
    { Ti_ = SamplingPeriod; };


    inline double NbSamplingsPreviewed() const
    { return N_; };
    inline void NbSamplingsPreviewed( double SamplingsPreviewed)
    { N_ = SamplingsPreviewed; };
    /// \}

    //
    // Private methods:
    //
  private:

    /// \brief Verify and eventually reduce the maximal acceleration of the hip joint necessary to attain the velocity reference in one sampling T_.
    /// The verification is based on the supposition that the final joint trajectory is composed by
    /// a fourth-order polynomial acceleration phase inside T_ and a constant velocity phase for the rest of the preview horizon.
    ///
    /// \param[in] Ref
    /// \param[in] CurrentSupport
    void verify_acceleration_hip_joint(const Reference &Ref,
        const SupportState &CurrentSupport);

    /// \brief Verify velocity of hip joint
    /// The velocity is verified only between previewed supports.
    /// The verification is based on the supposition that the final joint trajectory is a third-order polynomial.
    ///
    /// \param[in] Time
    /// \param[in] PreviewedSupportFoot
    /// \param[in] PreviewedSupportAngle
    /// \param[in] StepNumber
    /// \param[in] CurrentSupport
    /// \param[in] CurrentRightFootAngle
    /// \param[in] CurrentLeftFootAngle
    /// \param[in] CurrentLeftFootVelocity
    /// \param[in] CurrentRightFootVelocity
    void verify_velocity_hip_joint(double Time,
        double PreviewedSupportFoot,
        double PreviewedSupportAngle,
        unsigned StepNumber,
        const SupportState &CurrentSupport,
        double CurrentRightFootAngle,
        double CurrentLeftFootAngle,
        double CurrentLeftFootVelocity,
        double CurrentRightFootVelocity);

    /// \brief Verify angle of hip joint
    /// Reduce final velocity of the trunk if necessary
    ///
    /// \param[in] CurrentSupport
    /// \param[in] PreviewedTrunkAngleEnd
    /// \param[in] TrunkState
    /// \param[in] TrunkStateT
    /// \param[in] CurrentSupportAngle
    /// \param[in] StepNumber
    ///
    /// \return AngleOK
    bool verify_angle_hip_joint(const SupportState &CurrentSupport,
        double PreviewedTrunkAngleEnd,
        const COMState2 &TrunkState,
        COMState2 &TrunkStateT,
        double CurrentSupportFootAngle,
        unsigned StepNumber);

    //
    // Private members:
    //
  private:

    /// \brief Angular limitations of the hip joints
    double lLimitLeftHipYaw_, uLimitLeftHipYaw_, lLimitRightHipYaw_, uLimitRightHipYaw_; // TODO doublons? -> yes, but this is a temporary class

    /// \brief Maximal acceleration of a hip joint
    double uaLimitHipYaw_;

    /// \brief Upper crossing angle limit between the feet
    double uLimitFeet_;

    /// \brief Maximal velocity of a foot
    double uvLimitFoot_;

    /// \brief Single-support duration
    double SSPeriod_; //TODO doublon with MPCData -> yes, but this is a temporary class

    /// \brief Number of sampling in a preview window
    double N_;

    /// \brief Time between two samplings
    double T_;
    double Ti_;
    /// \brief Rotation sense of the trunks angular velocity and acceleration
    double signRotVelTrunk_, signRotAccTrunk_;

    /// \brief
    double SupportTimePassed_;

    /// \brief Numerical precision
    const static double EPS_;

    /// \brief Current trunk state
    COMState2 TrunkState_;
    /// \brief State of the trunk at the first previewed sampling
    COMState2 TrunkStateT_;



  };

}
#endif // MPC_WALKGEN_ORIENTATIONS_PREVIEW_H
