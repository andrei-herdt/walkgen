#include <mpc-walkgen/realclock.h>

#if (defined __WIN32__)
# include <Windows.h>
#elif defined __LINUX__
#include <stdint.h>
#include <unistd.h>
#define UINT64 uint64_t
#elif (defined __VXWORKS__)
#include <vxWorks.h>
#include <taskLib.h>
#include <sysLib.h>
#include <sched.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>
#define UINT64 uint64_t
#endif // __WIN32__

#include <iostream>

using namespace MPCWalkgen;


RealClock::RealClock():
num_counters_(0),  
num_max_counters_(0),
frequency_(1)
{}

RealClock::~RealClock(){}

void RealClock::ReserveMemory(int num_max_counters) {
	start_tick_vec_.reserve(num_max_counters);
	stop_tick_vec_.reserve(num_max_counters);
	total_ticks_vec_.reserve(num_max_counters);
	max_ticks_vec_.reserve(num_max_counters);

	num_max_counters_ = num_max_counters;
}
void RealClock::GetFrequency(unsigned long long milliseconds) {
#ifdef __WIN32__
	LARGE_INTEGER frequency;
	QueryPerformanceFrequency(&frequency);
	frequency_ = frequency.QuadPart;
#elif (defined __LINUX__ || defined __VXWORKS__) 
	IntegerType start_tick = __rdtsc();
#if (defined __VXWORKS__)
	double seconds = static_cast<double>(milliseconds) / 1000.0;
	taskDelay(static_cast<int>(seconds * sysClkRateGet()));
#elif defined __LINUX__
	usleep(milliseconds * 1000);
#endif
	frequency_ = (__rdtsc() - start_tick) / (milliseconds * 1000 );
#endif
}

int RealClock::StartCounter() {
	assert(num_counters_ < num_max_counters_);//TODO: Overflow possible?

#ifdef __WIN32__
	LARGE_INTEGER start_counter;
	QueryPerformanceCounter(&start_counter);
	start_tick_vec_.push_back(start_counter.QuadPart);
#elif (defined __LINUX__ || defined __VXWORKS__)
	start_tick_vec_.push_back(__rdtsc());
#endif

	if (num_counters_ >= static_cast<int>(stop_tick_vec_.size())) {
		stop_tick_vec_.resize(num_counters_ + 1);
	}
	if (num_counters_ >= static_cast<int>(total_ticks_vec_.size())) {
		total_ticks_vec_.push_back(0);
		max_ticks_vec_.push_back(0);
	}


	return num_counters_++;
}

void RealClock::StopCounter(int index) {
	assert(index > -1 && index <= num_counters_);//TODO: Overflow possible?

#ifdef __WIN32__
		LARGE_INTEGER stop_counter;
		QueryPerformanceCounter(&stop_counter);
		stop_tick_vec_.at(index) = stop_counter.QuadPart;
#elif (defined __LINUX__ || defined __VXWORKS__) 
		stop_tick_vec_.at(index) = __rdtsc();
#endif

		IntegerType ticks_diff = stop_tick_vec_.at(index) - start_tick_vec_.at(index);
		total_ticks_vec_.at(index) += ticks_diff;
		if (ticks_diff > max_ticks_vec_.at(index)) {
			max_ticks_vec_.at(index) = ticks_diff;
		}

}

void RealClock::StopLastCounter() {
	StopCounter(stop_tick_vec_.size() - 1);
}

double RealClock::PopBackTime() {
	if (!stop_tick_vec_.empty() && !start_tick_vec_.empty()) {
		double start_time = 0.0;
		double end_time = 0.0;
#ifdef __WIN32__
		start_time = start_tick_vec_.back() * (1000000.0 / frequency_);
		end_time = stop_tick_vec_.back() * (1000000.0 / frequency_);
#elif (defined __LINUX__ || defined __VXWORKS__) 
		start_time = static_cast<double>(start_tick_vec_.back()) / static_cast<double>(frequency_);
		end_time = static_cast<double>(stop_tick_vec_.back()) / static_cast<double>(frequency_);
#endif
		start_tick_vec_.pop_back();
		stop_tick_vec_.pop_back();
		--num_counters_;

		return (end_time - start_time);
	} else {
		return -2.0;
	}
}

double RealClock::GetTime(int index) {
	if (!stop_tick_vec_.empty() && !start_tick_vec_.empty()) {
		double start_time = 0.0;
		double end_time = 0.0;
#ifdef __WIN32__
		start_time = start_tick_vec_.at(index) * (1000000.0 / frequency_);
		end_time = stop_tick_vec_.at(index) * (1000000.0 / frequency_);
#elif (defined __LINUX__ || defined __VXWORKS__) 
		start_time = static_cast<double>(start_tick_vec_.at(index)) / static_cast<double>(frequency_);
		end_time = static_cast<double>(stop_tick_vec_.at(index)) / static_cast<double>(frequency_);
#endif

		return (end_time - start_time);
	} else {
		return -2.0;
	}
}

double RealClock::GetTotalTime(int counter) {
	if (counter < static_cast<int>(total_ticks_vec_.size()) && counter > -1) {
		double time = 0.0;
#ifdef __WIN32__
		time = total_ticks_vec_.at(counter) * (1000000.0 / frequency_);
#elif (defined __LINUX__ || defined __VXWORKS__) 
		time = static_cast<double>(total_ticks_vec_.at(counter)) / static_cast<double>(frequency_);
#endif
		return time;
	} else {
		return -2.0;
	}
}

double RealClock::PopBackTotalTime() {
	if (!total_ticks_vec_.empty()) {
		double time = 0.0;
#ifdef __WIN32__
		time = total_ticks_vec_.back() * (1000000.0 / frequency_);
#elif (defined __LINUX__ || defined __VXWORKS__) 
		time = static_cast<double>(total_ticks_vec_.back()) / static_cast<double>(frequency_);
#endif
		total_ticks_vec_.pop_back();

		return time;
	} else {
		return -2.0;
	}
}

double RealClock::GetMaxTime(int counter) {
	if (counter < static_cast<int>(max_ticks_vec_.size()) && counter > -1) {
		double time_period = 0.0;
#ifdef __WIN32__
		time_period = max_ticks_vec_.at(counter) * (1000000.0 / frequency_);
#elif (defined __LINUX__ || defined __VXWORKS__) 
		time_period = static_cast<double>(max_ticks_vec_.at(counter)) / static_cast<double>(frequency_);
#endif
		return time_period;
	} else {
		return -2.0;
	}
}

void RealClock::ResetLocal() {
	num_counters_ = 0;
	start_tick_vec_.resize(0);
	stop_tick_vec_.resize(0);
}

void RealClock::ResetTotal() {
	total_ticks_vec_.resize(0);
	max_ticks_vec_.resize(0);
}


//
// Private methods:
//
#if (defined __VXWORKS__) 
RealClock::IntegerType  RealClock::__rdtsc( void ){
	unsigned a, d;
	__asm__ volatile("rdtsc" : "=a" (a), "=d" (d));
	return ((IntegerType)a) | (((IntegerType)d) << 32);
}
#endif

#if defined __LINUX__
RealClock::IntegerType  RealClock::__rdtsc(void) {
    unsigned a, d;
    __asm__ __volatile__ ("rdtsc" : "=a"(a), "=d"(d));
    return ( static_cast<IntegerType>(a)) | ( (static_cast<IntegerType>(d))<<32 );
}
#endif
