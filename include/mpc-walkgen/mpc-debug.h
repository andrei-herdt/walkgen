#pragma once
#ifndef MPC_WALKGEN_MPC_DEBUG_H
#define MPC_WALKGEN_MPC_DEBUG_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	mpc-debug.h
////////////////////////////////////////////////////////////////////////////////

#include <map>
#ifdef WIN32
#include <windows.h> 
#endif // WIN32

namespace MPCWalkgen{

  enum TimeUnit { us, ms, s };

  class MPCDebug {

  public:
    MPCDebug();
    MPCDebug(bool enable);
    ~MPCDebug();

    void getTime(int id, bool start);
    double computeInterval(int id, TimeUnit unit = us);
    int nbIntervals(int id);

    void GetFrequency();
    void StartCounting();
    void StopCounting();
    double GetTime();

    void reset(int id);
    void reset();

  private:
    std::map<int,double> startTime_;
    std::map<int,double> endTime_;
    std::map<int,int> nbCount_;

    LONGLONG frequency_, first_counter_, last_counter_;

    bool enable_;

  };

}
#endif // MPC_WALKGEN_MPC_DEBUG_H
