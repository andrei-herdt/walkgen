#include <mpc-walkgen/mpc-debug.h>

#ifdef __WIN32__
# include <Windows.h>
#elif defined __LINUX__
#include <stdint.h>
#include <unistd.h>
#define UINT64 uint64_t
#elif defined __VXWORKS__
#include <vxWorks.h>
#include <taskLib.h>
#include <sysLib.h>
#include <sched.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>
#define UINT64 uint64_t
#endif // __WIN32__

using namespace MPCWalkgen;

MPCDebug::MPCDebug(const int num_max_counters):
curr_counter_index_(0),  
num_max_counters_(num_max_counters)
{
  last_counter_vec_.reserve(num_max_counters);
  first_counter_vec_.reserve(num_max_counters);
}

MPCDebug::~MPCDebug(){}

void MPCDebug::GetFrequency(unsigned long long mu_seconds) {
#ifdef __WIN32__
  QueryPerformanceFrequency(&frequency_);
#elif (defined __LINUX__ || defined __VXWORKS__) 
  first_counter_vec_.push_back(__rdtsc());
#ifdef __VXWORKS__
  double seconds = static_cast<double>(mu_seconds) / 1000000.0;
  taskDelay(static_cast<int>(seconds * sysClkRateGet()));
#endif
  frequency_ = (__rdtsc() - first_counter_vec_.back()) / mu_seconds;
  first_counter_vec_.pop_back();
#endif
}

int MPCDebug::StartCounting() {

  if (curr_counter_index_ >= num_max_counters_) {
    return -1;
  }

  first_counter_vec_.resize(curr_counter_index_ + 1);
  last_counter_vec_.resize(curr_counter_index_ + 1);
#ifdef __WIN32__
  QueryPerformanceCounter(&first_counter_vec_.back());
#elif (defined __LINUX__ || defined __VXWORKS__) 
  first_counter_vec_.push_back(__rdtsc());
#endif

  return curr_counter_index_++;
}

void MPCDebug::StopCounting(int counter_index) {
  if (counter_index > -1 && counter_index <= curr_counter_index_) {
#ifdef __WIN32__
    QueryPerformanceCounter(&last_counter_vec_[counter_index]);
#elif (defined __LINUX__ || defined __VXWORKS__) 
    last_counter_vec_[counter_index] = __rdtsc();
#endif
  }
}

double MPCDebug::GetLastMeasure() {
  if (!last_counter_vec_.empty() && !first_counter_vec_.empty()) {
    double start_time = 0.0;
    double end_time = 0.0;
#ifdef __WIN32__
     start_time = first_counter_vec_.back().QuadPart * (1000000.0 / frequency_.QuadPart);
     end_time = last_counter_vec_.back().QuadPart * (1000000.0 / frequency_.QuadPart);
#elif (defined __LINUX__ || defined __VXWORKS__) 
    start_time = static_cast<double>(first_counter_vec_.back()) / static_cast<double>(frequency_);
    end_time = static_cast<double>(last_counter_vec_.back()) / static_cast<double>(frequency_);
#endif
    first_counter_vec_.pop_back();
    last_counter_vec_.pop_back();
    --curr_counter_index_;

    return (end_time - start_time);
  } else {
    return -2.0;
  }
}

void MPCDebug::Reset() {
  curr_counter_index_ = 0;
  first_counter_vec_.resize(0);
  last_counter_vec_.resize(0);
}


//
// Private methods:
//
#if (defined __LINUX__ || defined __VXWORKS__) 
unsigned long long  MPCDebug::__rdtsc( void ){
  unsigned a, d;
  __asm__ volatile("rdtsc" : "=a" (a), "=d" (d));
  return ((UINT64)a) | (((UINT64)d) << 32);
}
#endif
