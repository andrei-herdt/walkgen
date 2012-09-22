#pragma once
#ifndef MPC_WALKGEN_QP_GENERATOR_H
#define MPC_WALKGEN_QP_GENERATOR_H

#include <mpc-walkgen/types.h>
#include <mpc-walkgen/qp-solver.h>
#include <mpc-walkgen/qp-preview.h>

#include <Eigen/Dense>
#include <vector>

namespace MPCWalkgen{
  class QPGenerator{

    //
    // Public methods:
    //
  public:
    QPGenerator(QPPreview *preview, QPSolver *solver,
      Reference *ref, WeightCoefficients *weight_coefficients,
      RigidBodySystem *robot, const MPCData *mpc_parameters);
    ~QPGenerator();

    void PrecomputeObjective();

    void BuildProblem(MPCSolution &solution);

    void computeWarmStart(MPCSolution &solution);

    void ConvertCopToJerk(MPCSolution &solution);

    void computeReferenceVector(const MPCSolution &solution);

    inline void current_time(double time) {current_time_ = time;};

  private:

    void BuildInequalitiesFeet(const MPCSolution &solution);

    void BuildObjective(const MPCSolution &solution);

    void buildConstraints(const MPCSolution &solution);

    void BuildConstraintsFeet(const MPCSolution &solution);

    void BuildFootVelConstraints(const MPCSolution &solution);

    void buildConstraintsCOP(const MPCSolution &solution);

  private:

    QPPreview *preview_;
    QPSolver *solver_;
    RigidBodySystem *robot_;
    Reference *vel_ref_, *pos_ref_;
    WeightCoefficients *weight_coefficients_;
    const MPCData *mpc_parameters_;

    CommonVectorType tmp_vec_, tmp_vec2_;
    CommonMatrixType tmp_mat_, tmp_mat2_;

    double current_time_;

    RelativeInequalities feetInequalities_;

    std::vector<CommonMatrixType> Qconst_;
    std::vector<CommonMatrixType> QconstN_;
    std::vector<CommonMatrixType> choleskyConst_;
    std::vector<CommonMatrixType> state_variant_;   //\brief These elements are multiplied by the state
    std::vector<CommonMatrixType> select_variant_;
    std::vector<CommonMatrixType> ref_variant_vel_;
    std::vector<CommonMatrixType> ref_variant_pos_;

    ConvexHull foot_hull_edges_;
    ConvexHull cop_hull_edges_;
    ConvexHull hull;


  };
}
#endif // MPC_WALKGEN_QP_GENERATOR_H
