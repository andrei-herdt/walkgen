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
    MPCDebug(int num_max_measures);
    ~MPCDebug();

    void GetFrequency(unsigned long long mu_seconds);
    
    int StartCounting();
    
    /// \brief Stop counter associated with counter_index
    void StopCounting(int counter_index);

    /// \brief Give the latest measured time and decrease size of vector
    double GetLastMeasure();
    
    inline int GetNumCounters() {return curr_counter_index_;};
    
    void Reset();

    //
    // Private methods:
    //
  private:
#if (defined __LINUX__ || defined __VXWORKS__)
    unsigned long long  __rdtsc( void );
#endif

    // 
    // Private members:
    //
  private:
#ifdef __WIN32__
    LARGE_INTEGER frequency_;
    std::vector<LARGE_INTEGER> last_counter_vec_;
    std::vector<LARGE_INTEGER> first_counter_vec_;
#elif (defined __LINUX__ || defined __VXWORKS__) 
    unsigned long long frequency_;
    std::vector<unsigned long long> last_counter_vec_;
    std::vector<unsigned long long> first_counter_vec_;
#endif

    int curr_counter_index_;
    const int num_max_counters_;

  };

}
#endif // MPC_WALKGEN_MPC_DEBUG_H
