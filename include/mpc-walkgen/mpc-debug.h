#pragma once
#ifndef MPC_WALKGEN_MPC_DEBUG_H
#define MPC_WALKGEN_MPC_DEBUG_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	mpc-debug.h
////////////////////////////////////////////////////////////////////////////////

#ifdef __WIN32__
#include <windows.h> 
#endif
#include <vector>

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
    double GetLastTimeValue();
	inline int GetNumMeasures() {return last_counter_vec_.size();};

  private:
#if (defined __LINUX__ || defined __VXWORKS__)
  unsigned long long  __rdtsc( void );
#endif

  private:
#ifdef __WIN32__
	  double frequency_;
	  std::vector<LONGLONG> last_counter_vec_;
	  std::vector<LONGLONG> first_counter_vec_;
#elif (defined __LINUX__ || defined __VXWORKS__) 
	  double frequency_;
	  std::vector<unsigned long long> last_counter_vec_;
	  std::vector<unsigned long long> first_counter_vec_;
#endif

  };

}
#endif // MPC_WALKGEN_MPC_DEBUG_H
