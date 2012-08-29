#pragma once
#ifndef MPC_WALKGEN_STOPWATCH_H
#define MPC_WALKGEN_STOPWATCH_H

///\file	stopwatch.h
///\brief	This class measures in musec the time passed between the calls
///			StartCounting() and StopCounting() by accessing the
///			time-stamp counter via __rtdsc()
///			Several counters can be started independently
///			Additionally, the total time measured by every counter is tracked
///
///\author	Herdt Andrei
///\author	Werner Alexander

#include <vector>
#if (defined __WIN32__ || defined _WIN32_)
#include <windows.h> 
#endif


namespace MPCWalkgen{

	class StopWatch {

		//
		// Public types
		//
	public:
#if (defined __WIN32__)
		typedef LARGE_INTEGER IntegerType;
#elif (defined __LINUX__ || defined __VXWORKS__ || defined _WIN32_)
		typedef unsigned long long IntegerType;
#endif
		//
		// Public methods:
		//
	public:
		StopWatch(int num_max_measures);
		~StopWatch();

		void GetFrequency(unsigned long long milliseconds);

		int StartCounter();

		/// \brief Stop counter associated with counter_index
		void StopCounter(int index);

		/// \brief Give the latest measured time and decrease size of vector
		double PopBackTime();
		double GetTime(int counter);

		/// \brief Give the total measured time of last counter and decrease size of vector
		double PopBackTotalTime();
		double GetTotalTime(int counter);

		inline int GetNumCounters() {return counter_num_;};
		inline int GetNumTotalCounters() {return total_ticks_vec_.size();};

		void Reset();
		void ResetTotal();

		//
		// Private methods:
		//
	private:
#if (defined __LINUX__ || defined __VXWORKS__)
		IntegerType  __rdtsc( void );
#elif defined __WIN64__
#include 
#elif defined _WIN32_
#define rdtsc	__asm __emit 0fh __asm __emit 031h
		inline unsigned __int64 __rdtsc();

#endif

		// 
		// Private data members:
		//
	private:
#ifdef __WIN32__
		IntegerType frequency_;
		std::vector<IntegerType> stop_tick_vec_;
		std::vector<IntegerType> start_tick_vec_;
#elif (defined __LINUX__ || defined __VXWORKS__ || defined _WIN32_) 
		IntegerType frequency_;
		std::vector<IntegerType> stop_tick_vec_;
		std::vector<IntegerType> start_tick_vec_;
#endif

		std::vector<IntegerType> total_ticks_vec_;		// Total num. ticks of every counter

		int counter_num_;
		const int num_max_counters_;

	};

}
#endif // MPC_WALKGEN_STOPWATCH_H
