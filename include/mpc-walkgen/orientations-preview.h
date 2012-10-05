#pragma once
#ifndef MPC_WALKGEN_ORIENTATIONS_PREVIEW_H
#define MPC_WALKGEN_ORIENTATIONS_PREVIEW_H

#include <mpc-walkgen/types.h>
#include <mpc-walkgen/rigid-body-system.h>

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

	void Init(const MPCParameters &mpc_parameters, const RobotData &robot_data);

	/// \brief Preview feet and trunk orientations inside the preview window.
	///
	/// The orientations of the feet are adapted to the previewed orientation of the hip.
	/// The resulting velocities accelerations and orientations are verified against the limits.
	/// If the constraints can not be satisfied the rotational velocity of the trunk is reduced.
	/// The trunk is rotating with a constant speed after a constant acceleration phase of T_ length.
	/// During the initial double support phase the trunk is not rotating contrary to the following.
	void preview_orientations(double Time,  /// \param[in]
			const Reference &Ref,    		/// \param[in]
			double StepDuration,    		/// \param[in]
			const BodyState &LeftFoot,    	/// \param[in]
			const BodyState &RightFoot,    	/// \param[in]
			MPCSolution &Solution    		/// \param[out] Solution Trunk and Foot orientations
	);

	/// \brief Interpolate previewed orientation of the trunk
	///
	/// \param[in] robot
	void InterpolateTrunkYaw(const RigidBodySystem * robot);

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
	void VerifyAccelerationHipYaw(const Reference &Ref,    /// \param[in]
			const SupportState &CurrentSupport    /// \param[in]
	);

	/// \brief Verify velocity of hip joint
	/// The velocity is verified only between previewed supports.
	/// The verification is based on the supposition that the final joint trajectory is a third-order polynomial.
	void VerifyVelocityHipYaw(double Time,    	/// \param[in]
			double PreviewedSupportFoot,    	/// \param[in]
			double PreviewedSupportAngle,    	/// \param[in]
			unsigned StepNumber,    			/// \param[in]
			const SupportState &CurrentSupport, /// \param[in]
			double CurrentRightFootAngle,    	/// \param[in]
			double CurrentLeftFootAngle,    	/// \param[in]
			double CurrentLeftFootVelocity,    	/// \param[in]
			double CurrentRightFootVelocity    	/// \param[in]
	);

	/// \brief Verify angle of hip joint.
	/// Reduce final velocity of the trunk if necessary.
	///
	/// \return AngleOK
	bool verify_angle_hip_joint(const SupportState &CurrentSupport,     /// \param[in]
			double PreviewedTrunkAngleEnd,    							/// \param[in]
			const COMState2 &TrunkState,    							/// \param[in]
			COMState2 &TrunkStateT,    									/// \param[in]
			double CurrentSupportFootAngle,    							/// \param[in]
			unsigned StepNumber    										/// \param[in]
	);

	//
	// Private members:
	//
private:

	/// \brief Angular limitations of the hip joints
	double lLimitLeftHipYaw_, uLimitLeftHipYaw_, lLimitRightHipYaw_, uLimitRightHipYaw_;

	double uaLimitHipYaw_;	/// \brief Maximal acceleration of a hip joint

	double uLimitFeet_;		/// \brief Upper crossing angle limit between the feet

	double uvLimitFoot_;	/// \brief Maximal velocity of a foot

	double SSPeriod_;		/// \brief Single-support duration

	double N_;				/// \brief Number of sampling in a preview window

	double T_;				/// \brief Time between two samplings
	double Ti_;

	double signRotVelTrunk_, signRotAccTrunk_;	/// \brief Rotation sense of the trunks angular velocity and acceleration

	double SupportTimePassed_;

	COMState2 TrunkState_;	/// \brief Current trunk state
	COMState2 TrunkStateT_;	/// \brief State of the trunk at the first previewed sampling



};

}
#endif // MPC_WALKGEN_ORIENTATIONS_PREVIEW_H
