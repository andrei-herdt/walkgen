#pragma once
#ifndef MPC_WALKGEN_SHAREDPGTYPE_H
#define  MPC_WALKGEN_SHAREDPGTYPE_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	api.h
///\brief	Definition of humanoid types
///\author  Herdt Andrei
///\author	Lafaye Jory
///\author      Keith François
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/common/sharedpgtypes.h>
#include <Eigen/Dense>
#include <vector>


namespace MPCWalkgen{

    /// \name Enum types
    /// \{
    enum Phase{
      SS, DS
    };

    enum Foot{
      LEFT, RIGHT
    };

    enum BodyType{
      LEFT_FOOT, RIGHT_FOOT, COM
    };
    /// \}

    /// \name Structures
    /// \{
    struct MPC_WALKGEN_API FootData{
      double soleWidth;
      double soleHeight;
      Eigen::Vector3d anklePositionInLocalFrame;

      FootData();
      FootData(const FootData &f);//TODO: LocalAnklePosition_ better?
      ~FootData();
    };

    struct MPC_WALKGEN_API HipYawData {
      double lowerBound;
      double upperBound;
      double lowerVelocityBound;
      double upperVelocityBound;
      double lowerAccelerationBound;
      double upperAccelerationBound;

      HipYawData();
      ~HipYawData();
    };

    struct MPC_WALKGEN_API QPPonderation{
      std::vector<double> instantVelocity;
      std::vector<double> CopCentering;
      std::vector<double> JerkMin;

      /// \brief Define the element of ponderation std::vector used in this iteration
      int activePonderation;

      QPPonderation(int nb = 2);
      ~QPPonderation();
    };

    struct MPC_WALKGEN_API SupportState {
      Phase phase;
      Foot foot;

      int nbStepsLeft;
      int stepNumber;
      int nbInstants;

      double time_limit;
      double start_time;

      double x, y, yaw;
      double yawTrunk;//TODO: Why in SupportState? -> for compatibility with temporary previewROrientation class

      bool state_changed;

      /// \brief Define if the support state is in a (transitional) double support phase
      bool transitional_ds;

      /// \brief The length of the previous sampling period (can be different from period_qpsample)
      double previousSamplingPeriod;//TODO: change name

      // \brief The relative weight of this support state in the QP (A support state duration of QPSamplingTime have : iterationWeight = 1)
      double sampleWeight;//TODO: shouldn't it be outside... somewhere...?
    };

    struct MPC_WALKGEN_API ConvexHull {
      /// \brief Set of vertices
      Eigen::VectorXd x;
      Eigen::VectorXd y;
      Eigen::VectorXd z;

      /// \brief Set of inequalities A*x+B*y+C*z+D>0
      Eigen::VectorXd A;
      Eigen::VectorXd B;
      Eigen::VectorXd C;
      Eigen::VectorXd D;

      ConvexHull();
      ~ConvexHull();

      ConvexHull &operator= (const ConvexHull &hull); // TODO: copyFrom() instead of =
      void resize(int size);
      void rotate(double yaw);
      void computeLinearSystem(const Foot &foot);
    };

    struct MPC_WALKGEN_API MPCData {
      // The following parameters are fixed once and for all at initialization
      /// \brief Sampling period considered in the QP
      double period_qpsample;    //blocked - precomputeObjective
      double period_mpcsample;   //blocked - precomputeObjective / RigidBodySystem::computeDynamicMatrix
      double period_actsample;   //blocked - precomputeObjective / RigidBodySystem::computeDynamicMatrix

      /// \brief Nb. samplings inside preview window
      int nbsamples_qp;  //blocked - precomputeObjective

      // \brief Step period ss_left<->ss_right in qp sample periods
      int nbqpsamples_step;  //blocked by orientPrw_ ? can be solved --
      // \brief Transition period ds->ss in qp sample periods
      int nbqpsamples_dsss;
      // \brief Steps to be done before ss->ds
      int nbsteps_ssds;
      // \brief Double support phase length (should be a large value)
      double period_ds;

      bool warmstart;
      /// \brief Interpolate not only the control (first element) but the whole preview vector
      bool interpolate_preview;


      QPPonderation ponderation;

      QPSolverType solver;
 
      /// \brief Compute the number of recomputations left until next sample
      int nbFeedbackSamplesLeft(double firstSamplingPeriod) const;
      /// \brief Number of simulation iterations between two feedback call
      int num_samples_act() const;
      /// \brief Number of feedback iterations between two QP instants
      int nbFeedbackSamplesStandard() const;

      int num_qpsamples_ss() const;
      int num_steps_max() const;

      double period_ss() const;
      double period_trans_ds() const;

      MPCData();
      ~MPCData();
    };

    struct MPC_WALKGEN_API RobotData {
      Eigen::Vector3d com;
      double freeFlyingFootMaxHeight;

      FootData leftFoot;
      FootData rightFoot;

      HipYawData leftHipYaw;
      HipYawData rightHipYaw;

      double robotMass; //TODO: rename mass

      Eigen::Vector3d leftFootPos;
      Eigen::Vector3d rightFootPos;

      ConvexHull leftFootHull;
      ConvexHull rightFootHull;
      ConvexHull CoPLeftSSHull;
      ConvexHull CoPRightSSHull;
      ConvexHull CoPLeftDSHull;
      ConvexHull CoPRightDSHull;

      RobotData(const FootData &leftFoot, const FootData &rightFoot,
        const HipYawData &leftHipYaw, const HipYawData &rightHipYaw,
        double mass);
      RobotData();
      ~RobotData();
    };

    struct Trajectory {
      Eigen::VectorXd x_vec, y_vec, z_vec, yaw_vec;
    };

    struct Motion {
      Trajectory pos, vel, acc, jerk;

      void resize(int size);
    };

    struct MPC_WALKGEN_API MPCSolution {

      Eigen::VectorXd qpSolution;
      Eigen::VectorXd initialSolution;

      Eigen::VectorXi constraints;
      Eigen::VectorXi initialConstraints;

      /// \brief True if a new trajectory is computed in online loop
      bool newTraj;

      /// \brief Sampling times
      /// starting with 0, i.e. all times are relative to the current time
      std::vector<double> samplingTimes_vec;

      std::vector<SupportState> support_states_vec;

      std::vector<double> supportOrientations_vec;//TODO: supportOrientations_vec
      std::vector<double> supportTrunkOrientations_vec;//TODO: TrunkOrientations_vec

      Eigen::VectorXd CoPTrajX;
      Eigen::VectorXd CoPTrajY;

      Motion com_prw, cop_prw, foot_left_prw, foot_right_prw; 
      Motion com_act, cop_act, foot_left_act, foot_right_act; 

      struct State {
        Eigen::VectorXd CoMTrajX_;
        Eigen::VectorXd CoMTrajY_;
        //TODO: Add CoMTrajZ_;

        Eigen::VectorXd leftFootTrajX_;
        Eigen::VectorXd leftFootTrajY_;
        Eigen::VectorXd leftFootTrajZ_;
        Eigen::VectorXd leftFootTrajYaw_;

        Eigen::VectorXd rightFootTrajX_;
        Eigen::VectorXd rightFootTrajY_;
        Eigen::VectorXd rightFootTrajZ_;
        Eigen::VectorXd rightFootTrajYaw_;

        Eigen::VectorXd trunkYaw_;
      };
      std::vector<State> state_vec;

      MPCSolution& operator = (MPCSolution const &);

      void reset();

      MPCSolution();
      ~MPCSolution();
    };

    struct MPC_WALKGEN_API StateValues {
      double x, dx, ddx;
      double y, dy, ddy;
      double z, dz, ddz;
      double yaw, dyaw, ddyaw;
    };

    struct MPC_WALKGEN_API ControlOutput {
      StateValues com, cop, left_foot, right_foot;
    };
}


#endif // MPC_WALKGEN_SHAREDPGTYPE_H
