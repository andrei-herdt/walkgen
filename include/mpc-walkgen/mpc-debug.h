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
#endif

namespace MPCWalkgen{

  //enum TimeUnit { us, ms, s };

class MPCDebug {

  public:
    MPCDebug();
    MPCDebug(bool enable);
    ~MPCDebug();

    void GetFrequency(double seconds);
    void StartCounting();
    void StopCounting();
    double GetTime();

  private:
#if (defined __LINUX__ || defined __VXWORKS__)
  unsigned long long  __rdtsc( void );
#endif

  private:
#ifdef __WIN32__
    LONGLONG frequency_, first_counter_, last_counter_;
#elif (defined __LINUX__ || defined __VXWORKS__) 
	  unsigned long long first_counter_, last_counter_, frequency_;
#endif

    bool enable_;

  };

}
#endif // MPC_WALKGEN_MPC_DEBUG_H
