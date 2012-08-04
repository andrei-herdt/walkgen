#pragma once
#ifndef MPC_WALKGEN_WALKGEN
#define MPC_WALKGEN_WALKGEN

////////////////////////////////////////////////////////////////////////////////
///
///\file	walkgen.h
///\author	Herdt Andrei
///\author	Lafaye Jory
///\author      Keith François
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/humanoid/walkgen-abstract.h>
#include "types.h"

namespace MPCWalkgen{

  class Interpolation;
  class QPSolver;
  class QPGenerator;
  class QPPreview;
  class RigidBodySystem;
  class OrientationsPreview;

  class Walkgen :
    public WalkgenAbstract
  {

  public:
    Walkgen(::MPCWalkgen::QPSolverType solvertype);
    ~Walkgen();

    virtual void init(const RobotData &robotData, const MPCData &mpcData);

    virtual void init();

    virtual const MPCSolution &online(double time);

    virtual const MPCSolution &online();

  public:
    // \name Accessors and mutators
    // \{
    void reference(double dx, double dy, double dyaw);
    void reference(Eigen::VectorXd dx, Eigen::VectorXd dy, Eigen::VectorXd dyaw);

    virtual const SupportState &currentSupportState() const;
    virtual inline void currentSupportState(const SupportState &newSupportState){
      newCurrentSupport_ = newSupportState;
      isNewCurrentSupport_ = true;
    }

    virtual const BodyState &bodyState(BodyType body)const;
    virtual void bodyState(BodyType body, const BodyState &state);

    virtual const ControlOutput &output() {
      return output_;
    };

    virtual RigidBodySystem *robot() {
      return robot_;
    }
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
    MPCData generalData_;
    RobotData robotData_;

    ::MPCWalkgen::QPSolver *solver_;
    QPGenerator *generator_;
    QPPreview *preview_;
    ::MPCWalkgen::Interpolation *interpolation_;
    RigidBodySystem *robot_;

    OrientationsPreview *orientPrw_;

    //States com, left_foot, right_foot;//TODO: Where to put this?

    // \brief Contains pointers to trajectory values
    // an internal logic set the pointers the currently relevant indeces
    ControlOutput output_;
    int output_index_;

    MPCSolution solution_;
    Reference velRef_;
    /// \brief The new value of reference velocity, updated with in online method
    Reference newVelRef_;
    QPPonderation ponderation_;
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
