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


#include "rigid-body-system.h"
#include "state-fsm.h"
#include "types.h"

#include <Eigen/Dense>


namespace MPCWalkgen{
    class QPPreview{
    private:
      static const double EPS_;

    public:
      QPPreview(Reference * velRef, RigidBodySystem * robot, const MPCData * generalData);

      ~QPPreview();

      void previewSamplingTimes(double currenttime, double firstSamplingPeriod, MPCSolution &solution);

      void previewSupportStates(double FirstIterationDynamicsDuration, MPCSolution &result);

      void computeRotationMatrix(MPCSolution &result);

      inline SelectionMatrices &selectionMatrices(){return selectionMatrices_;}

      inline const Eigen::MatrixXd &rotationMatrix() const{return rotationMatrix_;}

      inline const Eigen::MatrixXd &rotationMatrix2() const{return rotationMatrix2_;}

    private:

      void buildSelectionMatrices(MPCSolution & result);



    private:
      RigidBodySystem * robot_;
      const MPCData * generalData_;
      StateFSM * statesolver_;	//TODO: Name statesolver is bad

      SelectionMatrices selectionMatrices_;
      Eigen::MatrixXd rotationMatrix_;
      Eigen::MatrixXd rotationMatrix2_;

    };
}

/*! \fn MPCWalkgen::QPPreview::QPPreview(VelReference * velRef, RigidBodySystem * robot, const MPCData * generalData)
* \brief Constructor
*/

/*! \fn void MPCWalkgen::QPPreview::previewSupportStates(const double currentTime,
                                        const double FirstIterationDynamicsDuration,  MPCSolution & result)
* \brief Preview support states for the defined horizon
* \param [in] currentTime current time (synchronized with QP sampling time)
* \param [in] FirstIterationDynamicsDuration Duration of the first iteration
* \param [out] result
*/

/*! \fn SelectionMatrix & MPCWalkgen::QPPreview::selectionMatrices()
* \brief Return computed selection matrices
*/

#endif // MPC_WALKGEN_QP_PREVIEW_H
