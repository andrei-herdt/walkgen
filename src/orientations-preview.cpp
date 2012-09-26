#include <mpc-walkgen/orientations-preview.h>

#include <vector>

using namespace MPCWalkgen;
using namespace std;

namespace {

/// \brief Fourth order polynomial trajectory
/// \param[in] abcd Parameters
/// \param[in] x
/// \return Evaluation value
double f(double a,double b,double c,double d,double x)
{return a+b*x+c*x*x+d*x*x*x;}

/// \brief Fourth order polynomial trajectory derivative
/// \param[in] abcd Parameters
/// \param[in] x
/// \return Evaluation value
double df(double /*a*/,double b,double c,double d,double x)
{return b+2*c*x+3.0*d*x*x;}

} // end of anonymous namespace

OrientationsPreview::OrientationsPreview()
{ }

OrientationsPreview::~OrientationsPreview()
{ }

void OrientationsPreview::Init(const MPCParameters &data, const RobotData &robot_data) {
	T_ 			= data.period_qpsample;
	Ti_ 		= data.period_mpcsample;
	N_ 			= data.num_samples_horizon;
	SSPeriod_ 	= data.nbqpsamples_step * data.period_qpsample;//TODO: Incoherent

	lLimitLeftHipYaw_ = robot_data.left_hip_yaw.lowerBound;
	uLimitLeftHipYaw_ = robot_data.left_hip_yaw.upperBound;

	lLimitRightHipYaw_ = robot_data.right_hip_yaw.lowerBound;
	uLimitRightHipYaw_ = robot_data.right_hip_yaw.upperBound;

	//Velocity limit
	uvLimitFoot_ = fabs(robot_data.right_hip_yaw.upperVelocityBound);

	//Acceleration limit not given by HRP2JRLmain.wrl
	uaLimitHipYaw_ = robot_data.right_hip_yaw.upperAccelerationBound;

	//Maximal cross angle between the feet
	uLimitFeet_ = 5.0/180.0*M_PI;
}

void OrientationsPreview::preview_orientations(double Time,
		const Reference &Ref,
		double StepDuration,
		const BodyState &LeftFoot,
		const BodyState &RightFoot,
		MPCSolution &Solution) {

	const vector<SupportState> &PrwSupportStates_deq = Solution.support_states_vec;
	vector<double> &PreviewedSupportAngles_deq=Solution.support_yaw_vec;
	vector<double> &PreviewedTrunkOrientations_deq=Solution.trunk_yaw_vec;



	const SupportState &CurrentSupport = PrwSupportStates_deq.front();

	// Verify the acceleration of the hip joint
	VerifyAccelerationHipYaw(Ref, CurrentSupport);

	bool istrunk_vel_ok = false;
	bool istrunk_angle_ok = false;

	// In case of double support the next support angle is fixed
	// ds -> FirstFootPreviewed == 0
	// ss -> FirstFootPreviewed == 1
	double FirstFootPreviewed = 0.0;

	signRotVelTrunk_ = (TrunkStateT_.yaw[1] < 0.0)?-1.0:1.0;

	unsigned step = 0;

	// Parameters of the trunk polynomial (fourth order)
	double a,b,c,d,e;

	// Trunk angle at the end of the current support phase
	double PreviewedTrunkAngleEnd;

	while(!istrunk_vel_ok)
	{
		// Initialize support orientation:
		// -------------------------------
		double CurrentSupportAngle;
		if (CurrentSupport.foot == LEFT)
			CurrentSupportAngle = LeftFoot.yaw[0];
		else
			CurrentSupportAngle = RightFoot.yaw[0];


		// (Re)Compute the trunk orientation at the end of the acceleration phase:
		// -----------------------------------------------------------------------
		if(CurrentSupport.phase != DS)
		{
			istrunk_angle_ok = false;
			while(!istrunk_angle_ok)
			{
				if (fabs(TrunkStateT_.yaw[1]-TrunkState_.yaw[1]) > kEps)
				{
					a = TrunkState_.yaw[0];
					b = TrunkState_.yaw[1];
					c = 0.0;
					d = 3.0*(TrunkStateT_.yaw[1]-TrunkState_.yaw[1]) / (T_*T_);
					e = -2.0*d/(3.0*T_);
					TrunkStateT_.yaw[0] = a + b*T_+1.0/2.0*c*T_*T_+1.0/3.0*d*T_*T_*T_+1.0/4.0*e*T_*T_*T_*T_;
				} else {
					TrunkStateT_.yaw[0] = TrunkState_.yaw[0] + TrunkState_.yaw[1]*T_;
				}
				//Compute the trunk angle at the end of the support phase
				SupportTimePassed_ = CurrentSupport.time_limit - Time;
				PreviewedTrunkAngleEnd = TrunkStateT_.yaw[0] + TrunkStateT_.yaw[1]*(SupportTimePassed_-T_);

				//Verify the angle between the support foot and the trunk at the end of the current support period
				istrunk_angle_ok = verify_angle_hip_joint(CurrentSupport, PreviewedTrunkAngleEnd, TrunkState_, TrunkStateT_, CurrentSupportAngle, step);
			}
		} else {//The trunk does not rotate in the DS phase
			SupportTimePassed_ = CurrentSupport.time_limit+SSPeriod_-Time;
			FirstFootPreviewed = 1;
			PreviewedSupportAngles_deq.push_back(CurrentSupportAngle);
			TrunkStateT_.yaw[0] = PreviewedTrunkAngleEnd = TrunkState_.yaw[0];
		}

		// Initialize variables in the orientations preview loop:
		// ------------------------------------------------------
		double PreviousSupportAngle = CurrentSupportAngle;
		double PreviewedSupportFoot;
		if(CurrentSupport.foot == LEFT) {
			PreviewedSupportFoot = 1.0;
		} else {
			PreviewedSupportFoot = -1.0;
		}
		double CurrentLeftFootAngle = LeftFoot.yaw[0];
		double CurrentRightFootAngle = RightFoot.yaw[0];
		double CurrentLeftFootVelocity = LeftFoot.yaw[1];
		double CurrentRightFootVelocity = RightFoot.yaw[1];

		// Preview of orientations:
		// ------------------------
		for(step = (unsigned) FirstFootPreviewed; step <= (unsigned)((int)ceil((N_+1)*T_/StepDuration)); step++) {
			PreviewedSupportFoot = -PreviewedSupportFoot;
			//compute the optimal support orientation
			double PreviewedSupportAngle = PreviewedTrunkAngleEnd + TrunkStateT_.yaw[1]*SSPeriod_/2.0;

			VerifyVelocityHipYaw(Time,
					PreviewedSupportFoot, PreviewedSupportAngle,
					step, CurrentSupport,
					CurrentRightFootAngle, CurrentLeftFootAngle,
					CurrentLeftFootVelocity, CurrentRightFootVelocity);

			//Check the feet angles to avoid self-collision:
			if ((double)PreviewedSupportFoot*(PreviousSupportAngle-PreviewedSupportAngle)-kEps > uLimitFeet_) {
				PreviewedSupportAngle = PreviousSupportAngle+(double)signRotVelTrunk_*uLimitFeet_;
			} else if (fabs(PreviewedSupportAngle-PreviousSupportAngle) > uvLimitFoot_*SSPeriod_) {
				//not being able to catch-up for a rectangular DS phase

				PreviewedSupportAngle = PreviousSupportAngle+(double)PreviewedSupportFoot * uvLimitFoot_*(SSPeriod_-T_);
			}

			// Verify orientation of the hip joint at the end of the support phase
			istrunk_angle_ok = verify_angle_hip_joint( CurrentSupport, PreviewedTrunkAngleEnd,
					TrunkState_, TrunkStateT_,
					CurrentSupportAngle, step);

			if(!istrunk_angle_ok) {
				PreviewedSupportAngles_deq.clear();
				istrunk_vel_ok = false;
				break;
			} else {
				PreviewedSupportAngles_deq.push_back(PreviewedSupportAngle);
			}

			//Prepare for the next step
			PreviewedTrunkAngleEnd = PreviewedTrunkAngleEnd + SSPeriod_*TrunkStateT_.yaw[1];
			PreviousSupportAngle = PreviewedSupportAngle;

			if(static_cast<int>(PreviewedSupportFoot) == 1)
				CurrentLeftFootAngle = PreviewedSupportAngle;
			else
				CurrentRightFootAngle = PreviewedSupportAngle;

			istrunk_vel_ok = true;
		}

	}

	// PREVIEW TRUNK AND SUPPORT ORIENTATIONS:
	// ---------------------------------------
	PreviewedTrunkOrientations_deq.push_back(TrunkState_.yaw[0]);
	PreviewedTrunkOrientations_deq.push_back(TrunkStateT_.yaw[0]);
	unsigned j = 0;
	for(unsigned i = 1; i<N_; i++ ) {
		PreviewedTrunkOrientations_deq.push_back(TrunkStateT_.yaw[0]+TrunkStateT_.yaw[1]*T_);
	}

	std::vector<SupportState>::iterator prwSS_it = Solution.support_states_vec.begin();
	double supportAngle = prwSS_it->yaw;
	prwSS_it++;//Point at the first previewed instant
	for(unsigned i = 0; i<N_; i++ ) {
		if(prwSS_it->state_changed) {
			supportAngle = Solution.support_yaw_vec[j];
			j++;
		}
		prwSS_it->yaw = supportAngle;
		prwSS_it++;
	}

}


void OrientationsPreview::VerifyAccelerationHipYaw(const Reference &Ref,
		const SupportState &CurrentSupport) {
	if(CurrentSupport.phase != DS)
		//Verify change in velocity reference against the maximal acceleration of the hip joint
		if(fabs(Ref.local.yaw(1)-TrunkState_.yaw[1]) > 2.0/3.0*T_*uaLimitHipYaw_)
		{
			double signRotAccTrunk = (Ref.local.yaw(1)-TrunkState_.yaw[1] < 0.0)?-1.0:1.0;
			TrunkStateT_.yaw[1] = TrunkState_.yaw[1] + signRotAccTrunk * 2.0/3.0*T_* uaLimitHipYaw_;
		}
		else
			TrunkStateT_.yaw[1] = Ref.local.yaw(1);
	else//No rotations in a double support phase
		TrunkStateT_.yaw[1] = 0.0;
}

bool OrientationsPreview::verify_angle_hip_joint(const SupportState &CurrentSupport,
		double PreviewedTrunkAngleEnd,
		const COMState2 &TrunkState, COMState2 &TrunkStateT,
		double CurrentSupportFootAngle,
		unsigned StepNumber) {

	//Which limitation is relevant in the current situation?
	double uJointLimit, lJointLimit, JointLimit;
	if(CurrentSupport.foot == LEFT) {
		uJointLimit = uLimitLeftHipYaw_;
		lJointLimit = lLimitLeftHipYaw_;
	} else {
		uJointLimit = uLimitRightHipYaw_;
		lJointLimit = lLimitRightHipYaw_;
	}
	JointLimit = (TrunkStateT.yaw[1] < 0.0)?lJointLimit:uJointLimit;

	// Determine a new orientation if limit violated
	if (fabs(PreviewedTrunkAngleEnd - CurrentSupportFootAngle)>fabs(JointLimit)) {
		TrunkStateT.yaw[1] = (CurrentSupportFootAngle+0.9*JointLimit-TrunkState.yaw[0]-TrunkState.yaw[1]*T_/2.0)/(SupportTimePassed_+StepNumber*SSPeriod_-T_/2.0);
		return false;
	} else {
		return true;
	}
}


void OrientationsPreview::VerifyVelocityHipYaw(double Time,
		double previewed_supp_foot, double previewed_supp_angle,
		unsigned StepNumber, const SupportState &CurrentSupport,
		double CurrentRightFootAngle, double CurrentLeftFootAngle,
		double curr_left_foot_vel, double curr_right_foot_vel) {
	double curr_angle;
	if (static_cast<int>(previewed_supp_foot) == 1) {
		curr_angle = CurrentLeftFootAngle;
	} else {
		curr_angle = CurrentRightFootAngle;
	}

	// Parameters
	double a,b,c,d,T;

	//To be implemented
	//For the
	if(StepNumber>0 && CurrentSupport.phase == SS) {
		//verify the necessary, maximal, relative foot velocity
		double MeanFootVelDifference = (previewed_supp_angle-curr_angle)/(SSPeriod_-T_);
		//If necessary reduce the velocity to the maximum
		if (3.0/2.0*fabs(MeanFootVelDifference) > uvLimitFoot_)
		{
			MeanFootVelDifference = 2.0/3.0*(double)signRotVelTrunk_ * uvLimitFoot_;
			//Compute the resulting angle
			previewed_supp_angle = curr_angle + MeanFootVelDifference*(SSPeriod_-T_);
		}
	}
	else if((StepNumber==0 && CurrentSupport.phase == SS) || (StepNumber==1 && CurrentSupport.phase == DS)) {
		T = CurrentSupport.time_limit-Time-T_;
		//Previewed polynome
		a = curr_angle;
		if(static_cast<int>(previewed_supp_foot) == 1) {
			b = curr_left_foot_vel;
		} else {
			b = curr_right_foot_vel;
		}
		c = (3.0*previewed_supp_angle-3.0*a-2.0*b*T)/(T*T);
		d = (-b*T+2*a-2*previewed_supp_angle)/(T*T*T);

		//maximal speed violated
		if (df(a,b,c,d,-1.0/3.0*c/d)>uvLimitFoot_) {
			a = 0;
			c = -1.0/(2.0*T)*(2.0*b-2.0*uvLimitFoot_+2.0*sqrt(uvLimitFoot_*uvLimitFoot_-b*uvLimitFoot_));
			d = (-2.0*c-b/T)/(3.0*T);
			previewed_supp_angle = f(a,b,c,d,T);
		}
	}

}


void
OrientationsPreview::InterpolateTrunkYaw(const RigidBodySystem * robot)
{
	TrunkState_.yaw[0] = robot->body(COM)->state().yaw[0];
	TrunkState_.yaw[1] = robot->body(COM)->state().yaw[1];
	TrunkState_.yaw[2] = robot->body(COM)->state().yaw[2];
}

