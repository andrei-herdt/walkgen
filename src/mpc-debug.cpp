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

MPCDebug::MPCDebug()
{
	last_counter_vec_.reserve(20);
	first_counter_vec_.reserve(20);
}

MPCDebug::~MPCDebug(){}

void MPCDebug::GetFrequency(double seconds) {
#ifdef __WIN32__
  QueryPerformanceFrequency((LARGE_INTEGER*)&frequency_);
#elif (defined __LINUX__ || defined __VXWORKS__) 
  first_counter_vec_.push_back(__rdtsc());
#ifdef __VXWORKS__
 taskDelay(static_cast<int>(seconds * sysClkRateGet()));
#elif __LINUX__
  usleep(100);
#endif
  frequency_ = static_cast<double>(__rdtsc() - first_counter_vec_.back()) / seconds;
  first_counter_vec_.pop_back();
#endif
}

void MPCDebug::StartCounting() {
#ifdef __WIN32__
	first_counter_vec_.push_back(0);
  QueryPerformanceCounter((LARGE_INTEGER*)&first_counter_vec_.back());
#elif (defined __LINUX__ || defined __VXWORKS__) 
  first_counter_vec_.push_back(__rdtsc());
#endif
}

void MPCDebug::StopCounting() {
#ifdef __WIN32__
	last_counter_vec_.push_back(0);
  QueryPerformanceCounter((LARGE_INTEGER*)&last_counter_vec_.back());
#elif (defined __LINUX__ || defined __VXWORKS__) 
  last_counter_vec_.push_back(__rdtsc());
#endif
}

double MPCDebug::GetLastTimeValue() {
	if (!last_counter_vec_.empty() && !first_counter_vec_.empty()) {
		return( static_cast<double>(last_counter_vec_.back() - first_counter_vec_.back()) 
			/ frequency_ * 1000000.0 );
		last_counter_vec_.pop_back();
		first_counter_vec_.pop_back();
	} else {
		return -1.0;
	}
}

//
// private:
//
#if (defined __LINUX__ || defined __VXWORKS__) 
unsigned long long  MPCDebug::__rdtsc( void ){
	unsigned a, d;
	__asm__ volatile("rdtsc" : "=a" (a), "=d" (d));
	return ((UINT64)a) | (((UINT64)d) << 32);
}
#endif
