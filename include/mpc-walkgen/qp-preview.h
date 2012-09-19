#pragma once
#ifndef MPC_WALKGEN_QP_PREVIEW_H
#define MPC_WALKGEN_QP_PREVIEW_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	qp-preview.h
///\brief	A class to compute preview support state and selection matrices
///\author	Herdt Andrei
///\author      Keith François
///\author	Lafaye Jory
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////

#include <mpc-walkgen/rigid-body-system.h>
#include <mpc-walkgen/state-fsm.h>
#include <mpc-walkgen/types.h>

#include <Eigen/Dense>

namespace MPCWalkgen{
    class QPPreview{

      //
      // Public methods:
      //
    public:
      QPPreview(Reference *ref, RigidBodySystem *robot, const MPCData *mpc_parameters);
      ~QPPreview();

      void previewSamplingTimes(double currenttime, double firstSamplingPeriod, MPCSolution &solution);

      void previewSupportStates(double FirstIterationDynamicsDuration, MPCSolution &solution);

      void computeRotationMatrix(MPCSolution &solution);

      inline SelectionMatrices &selectionMatrices(){return selectionMatrices_;}

      inline const CommonMatrixType &rotationMatrix() const{return rotationMatrix_;}

      inline const CommonMatrixType &rotationMatrix2() const{return rotationMatrix2_;}

      //
      // Private methods:
      //
    private:
      void buildSelectionMatrices(MPCSolution &solution);

      //
      // Private data members:
      //
    private:
      RigidBodySystem * robot_;
      const MPCData * mpc_parameters_;
      StateFSM * statesolver_;	//TODO: Name statesolver is bad

      SelectionMatrices selectionMatrices_;
      CommonMatrixType rotationMatrix_;
      CommonMatrixType rotationMatrix2_;

    private:
      static const double EPS_;
    };
}

#endif // MPC_WALKGEN_QP_PREVIEW_H
