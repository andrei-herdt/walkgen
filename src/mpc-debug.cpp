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

MPCDebug::MPCDebug() {};
MPCDebug::MPCDebug(bool enable)
:enable_(enable)
{}//TODO: Remove enable_

MPCDebug::~MPCDebug(){}

void MPCDebug::GetFrequency(double seconds) {
#ifdef __WIN32__
  QueryPerformanceFrequency((LARGE_INTEGER*)&frequency_);
#elif (defined __LINUX__ || defined __VXWORKS__) 
  unsigned long long first_counter_ = __rdtsc();
#ifdef __VXWORKS__
 taskDelay(static_cast<int>(seconds * sysClkRateGet()));
#elif __LINUX__
  usleep(100);
#endif
  frequency_ = ( __rdtsc() - first_counter_ ) / seconds;
  first_counter_ = __rdtsc();
#endif
}

void MPCDebug::StartCounting() {
#ifdef __WIN32__
  QueryPerformanceCounter((LARGE_INTEGER*)&first_counter_);
#elif (defined __LINUX__ || defined __VXWORKS__) 
  first_counter_ = __rdtsc();
#endif
}

void MPCDebug::StopCounting() {
#ifdef __WIN32__
  QueryPerformanceCounter((LARGE_INTEGER*)&last_counter_);
#elif (defined __LINUX__ || defined __VXWORKS__) 
  last_counter_ = __rdtsc();
#endif
}

double MPCDebug::GetTime() {
  return( static_cast<double>(last_counter_ - first_counter_) / (static_cast<double>(frequency_)/1000000.0) );
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
