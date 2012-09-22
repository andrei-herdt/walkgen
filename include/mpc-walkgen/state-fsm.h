#pragma once
#ifndef MPC_WALKGEN_STATE_FSM_H
#define MPC_WALKGEN_STATE_FSM_H

#include <mpc-walkgen/types.h>

namespace MPCWalkgen{
    class StateFSM{
    public:
      StateFSM(Reference *ref, const MPCData *mpc_parameters);
      ~StateFSM();

      void setSupportState(int sample, const std::vector<double> &samplingTimes_vec, SupportState &Support);

    protected:
      Reference *vel_ref_;
      const MPCData *mpc_parameters_;

    };
}

#endif // MPC_WALKGEN_STATE_FSM_H
