#pragma once
#ifndef MPC_WALKGEN_WALKGEN
#define MPC_WALKGEN_WALKGEN

////////////////////////////////////////////////////////////////////////////////
///
///\file	walkgen.h
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author      Keith François
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/types.h>
#include <mpc-walkgen/realclock.h>

#include <mpc-walkgen/orientations-preview.h>
#include <mpc-walkgen/qp-solver.h>
#include <mpc-walkgen/qp-generator.h>
#include <mpc-walkgen/qp-preview.h>
#include <mpc-walkgen/rigid-body-system.h>
#include <mpc-walkgen/realclock.h>

namespace MPCWalkgen{

  class Walkgen {
  public:
    Walkgen();
    ~Walkgen();

    void Init(const MPCData &mpc_parameters);
    void Init(const RobotData &data_robot);

    void Init();

    const MPCSolution &online(double time);
    const MPCSolution &online();

  public:
    // \name Accessors and mutators
    // \{
    void reference(double dx, double dy, double dyaw);
    void reference(Eigen::VectorXd dx, Eigen::VectorXd dy, Eigen::VectorXd dyaw);

    const SupportState &currentSupportState() const;
    inline void currentSupportState(const SupportState &newSupportState){
      newCurrentSupport_ = newSupportState;
      isNewCurrentSupport_ = true;
    }

    const BodyState &bodyState(BodyType body) const;
    void bodyState(BodyType body, const BodyState &state);
    const ControlOutput &output() const { return output_; };
    RigidBodySystem *robot() { return robot_; };

    const QPSolver *solver() const { return solver_; };

    RealClock &clock() {return clock_;};

    const MPCSolution &solution() {return solution_;};
    // \}

  private:
    void BuildProblem();
    void GenerateTrajectories();
    void ResetOutputIndex();
    void IncrementOutputIndex();
    void UpdateOutput();
    void ResetCounters(double time);
    void SetCounters(double time);

  private:
    MPCData mpc_parameters_;
    RobotData robotData_;

    ::MPCWalkgen::QPSolver *solver_;
    QPGenerator *generator_;
    QPPreview *preview_;
    RigidBodySystem *robot_;

    OrientationsPreview *orientPrw_;

    RealClock clock_;

    // \brief Contains pointers to trajectory values
    // an internal logic set the pointers the currently relevant indeces
    ControlOutput output_;
    int output_index_;

    MPCSolution solution_;
    Reference velRef_;
    /// \brief The new value of reference velocity, updated with in online method
    Reference newVelRef_;
    WeightCoefficients weight_coefficients_;

    /// \brief The new value of current support state, updated with in online method
    SupportState newCurrentSupport_;
    bool isNewCurrentSupport_;

    /// \brief Time at which the problem should be updated
    double first_sample_time_;
    double next_computation_;
    double next_act_sample_;

    /// \brief Synchronised time with QP sampling
    double currentTime_;
    double currentRealTime_;
  };
}



#endif // MPC_WALKGEN_WALKGEN
