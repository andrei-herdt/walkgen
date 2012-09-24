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
#include <mpc-walkgen/realclock.h>//TODO: Add in Makefile

namespace MPCWalkgen{
    class QPPreview{

      //
      // Public methods:
      //
    public:
      QPPreview(Reference *ref, RigidBodySystem *robot,
    		  const MPCData *mpc_parameters,
    		  RealClock *clock);
      ~QPPreview();

      void previewSamplingTimes(double current_time, double firstSamplingPeriod, MPCSolution &solution);

      void previewSupportStates(double FirstIterationDynamicsDuration, MPCSolution &solution);

      void computeRotationMatrix(MPCSolution &solution);

      inline SelectionMatrices &selectionMatrices(){return select_matrices_;}

      inline const CommonMatrixType &rotationMatrix() const{return rotationMatrix_;}
      inline const CommonMatrixType &rotationMatrix2() const{return rotationMatrix2_;}
      inline const CommonMatrixType &rotationMatrix2Trans() const{return rotationMatrix2Trans_;}
      //inline const SparseMatrixType &sparse_rot_mat2() const{return sparse_rot_mat2_;}
      //inline const SparseMatrixType &sparse_rot_mat2_trans() const{return sparse_rot_mat2_trans_;}


      //
      // Private methods:
      //
    private:
      void buildSelectionMatrices(MPCSolution &solution);

      //
      // Private data members:
      //
    private:
      RigidBodySystem *robot_;
      const MPCData *mpc_parameters_;
      StateFSM *statesolver_;	//TODO: Name statesolver is bad

      SelectionMatrices select_matrices_;

      CommonMatrixType rotationMatrix_;
      CommonMatrixType rotationMatrix2_;
      CommonMatrixType rotationMatrix2Trans_;

      //std::vector<TripletType> triplet_list1_, triplet_list2_;
      //SparseMatrixType sparse_rot_mat2_, sparse_rot_mat2_trans_;

      RealClock *clock_;

    };
}

#endif // MPC_WALKGEN_QP_PREVIEW_H
