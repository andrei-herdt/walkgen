#pragma once
#ifndef MPC_WALKGEN_STATE_FSM_H
#define MPC_WALKGEN_STATE_FSM_H

#include <mpc-walkgen/types.h>

namespace MPCWalkgen{
    class StateFSM{//TODO: change name
    public:
      StateFSM(Reference *ref, const MPCParameters *mpc_parameters);
      ~StateFSM();

      void SetSupportState(int sample, const std::vector<double> &samplingTimes_vec, SupportState &Support);

    protected:
      Reference *vel_ref_;
      const MPCParameters *mpc_parameters_;

    };
}

#endif // MPC_WALKGEN_STATE_FSM_H
