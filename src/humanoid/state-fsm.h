#pragma once
#ifndef MPC_WALKGEN_STATE_FSM_H
#define MPC_WALKGEN_STATE_FSM_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	state-fsm.h
///\brief	A abstract class to regroup all state-fsms
///\author	Herdt Andrei
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////

#include "types.h"

namespace MPCWalkgen{
    class StateFSM{
    public:
      StateFSM(Reference * velRef, const MPCData * generalData);
      ~StateFSM();

      void setSupportState(int sample, const std::vector<double> &samplingTimes_vec, SupportState &Support);

    protected:
      Reference * velRef_;
      const MPCData * generalData_;

    };
}

#endif // MPC_WALKGEN_STATE_FSM_H