#include <mpc-walkgen/stopwatch.h>

#if (defined __WIN32__ || defined _WIN32_)
# include <Windows.h>
#elif defined __LINUX__
#include <stdint.h>
#include <unistd.h>
#define UINT64 uint64_t
#elif (defined __VXWORKS__ || defined _WIN32_)
#include <vxWorks.h>
#include <taskLib.h>
#include <sysLib.h>
#include <sched.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>
#define UINT64 uint64_t
#endif // __WIN32__

#include<iostream.h>

using namespace MPCWalkgen;

StopWatch::StopWatch(const int num_max_counters):
counter_num_(0),  
num_max_counters_(num_max_counters)
{
	start_tick_vec_.reserve(num_max_counters);
	stop_tick_vec_.reserve(num_max_counters);
}

StopWatch::~StopWatch(){}

void StopWatch::GetFrequency(unsigned long long milliseconds) {
#ifdef __WIN32__
	QueryPerformanceFrequency(&frequency_);
#elif (defined __LINUX__ || defined __VXWORKS__ || defined _WIN32_) 
	start_tick_vec_.push_back(__rdtsc());
#if (defined __VXWORKS__)
	double seconds = static_cast<double>(milliseconds) / 1000.0;
	taskDelay(static_cast<int>(seconds * sysClkRateGet()));
#elif defined _WIN32_
	Sleep(static_cast<DWORD>(milliseconds));
#endif
	frequency_ = (__rdtsc() - start_tick_vec_.back()) / milliseconds / 1000.0 ;
	start_tick_vec_.pop_back();
#endif
}

int StopWatch::StartCounter() {
	if (counter_num_ >= num_max_counters_) {
		return -1;
	}

	stop_tick_vec_.resize(counter_num_ + 1);
	if (counter_num_ >= static_cast<int>(total_ticks_vec_.size())) {
		total_ticks_vec_.push_back(0);
	}
	
#ifdef __WIN32__
	QueryPerformanceCounter(&start_tick_vec_.back());
#elif (defined __LINUX__ || defined __VXWORKS__ || defined _WIN32_) 
	start_tick_vec_.push_back(__rdtsc());
#endif

	return counter_num_++;
}

void StopWatch::StopCounter(int index) {
	if (index > -1 && index <= counter_num_) {
#ifdef __WIN32__
		QueryPerformanceCounter(&stop_tick_vec_.at(index));
#elif (defined __LINUX__ || defined __VXWORKS__ || defined _WIN32_) 
		stop_tick_vec_.at(index) = __rdtsc();
#endif

		IntegerType counter_diff = stop_tick_vec_.at(index) - start_tick_vec_.at(index);
		total_ticks_vec_.at(index) += counter_diff;
	}
}

double StopWatch::PopBackTime() {
	if (!stop_tick_vec_.empty() && !start_tick_vec_.empty()) {
		double start_time = 0.0;
		double end_time = 0.0;
#ifdef __WIN32__
		start_time = start_tick_vec_.back().QuadPart * (1000000.0 / frequency_.QuadPart);
		end_time = stop_tick_vec_.back().QuadPart * (1000000.0 / frequency_.QuadPart);
#elif (defined __LINUX__ || defined __VXWORKS__ || defined _WIN32_) 
		start_time = static_cast<double>(start_tick_vec_.back()) / static_cast<double>(frequency_);
		end_time = static_cast<double>(stop_tick_vec_.back()) / static_cast<double>(frequency_);
#endif
		start_tick_vec_.pop_back();
		stop_tick_vec_.pop_back();
		--counter_num_;

		return (end_time - start_time);
	} else {
		return -2.0;
	}
}

double StopWatch::GetTime(int index) {
	if (!stop_tick_vec_.empty() && !start_tick_vec_.empty()) {
		double start_time = 0.0;
		double end_time = 0.0;
#ifdef __WIN32__
		start_time = start_tick_vec_.at(index).QuadPart * (1000000.0 / frequency_.QuadPart);
		end_time = stop_tick_vec_.at(index).QuadPart * (1000000.0 / frequency_.QuadPart);
#elif (defined __LINUX__ || defined __VXWORKS__ || defined _WIN32_) 
		start_time = static_cast<double>(start_tick_vec_.at(index)) / static_cast<double>(frequency_);
		end_time = static_cast<double>(stop_tick_vec_.at(index)) / static_cast<double>(frequency_);
#endif

		return (end_time - start_time);
	} else {
		return -2.0;
	}
}

double StopWatch::GetTotalTime(int counter) {
	if (counter < total_ticks_vec_.size() && counter > -1) {
		double time = 0.0;
#ifdef __WIN32__
		time = total_ticks_vec_.at(counter).QuadPart * (1000000.0 / frequency_.QuadPart);
#elif (defined __LINUX__ || defined __VXWORKS__ || defined _WIN32_) 
		time = static_cast<double>(total_ticks_vec_.at(counter)) / static_cast<double>(frequency_);
#endif
		return time;
	} else {
		return -2.0;
	}
}

double StopWatch::PopBackTotalTime() {
	if (!total_ticks_vec_.empty()) {
		double time = 0.0;
#ifdef __WIN32__
		time = total_ticks_vec_.back().QuadPart * (1000000.0 / frequency_.QuadPart);
#elif (defined __LINUX__ || defined __VXWORKS__ || defined _WIN32_) 
		time = static_cast<double>(total_ticks_vec_.back()) / static_cast<double>(frequency_);
#endif
		total_ticks_vec_.pop_back();

		return time;
	} else {
		return -2.0;
	}
}


void StopWatch::Reset() {
	counter_num_ = 0;
	start_tick_vec_.resize(0);
	stop_tick_vec_.resize(0);
}

void StopWatch::ResetTotal() {
	total_ticks_vec_.resize(0);
}


//
// Private methods:
//
#if (defined __LINUX__ || defined __VXWORKS__) 
StopWatch::IntegerType  StopWatch::__rdtsc( void ){
	unsigned a, d;
	__asm__ volatile("rdtsc" : "=a" (a), "=d" (d));
	return ((UINT64)a) | (((UINT64)d) << 32);
}
#endif
#if defined _WIN32_
inline unsigned __int64 __rdtsc()
{
	IntegerType	li;

	rdtsc;

	__asm	mov	li.LowPart, eax;
	__asm	mov	li.HighPart, edx;
	return li.QuadPart;
}

#endif
