#pragma once
#ifndef MPC_WALKGEN_REALCLOCK_H
#define MPC_WALKGEN_REALCLOCK_H

///\file	realclock.h
///\brief	This class measures in musec the time passed between the calls
///			StartCounting() and StopCounting().
///			Under Vxworks and Linux, the Time Stamp Counter is called via __rtdsc()
///			Under Windows, (the more precise) QueryPerformanceCounter is used.
///			Several (local) counters can be started and stopped independently.
///			Additionally, the total and maximal time measured by every counter is tracked
///			by global counters.
///
///\author	Herdt Andrei
///\author	Werner Alexander

#include <vector>
#if (defined __WIN32__)
#include <windows.h> 
#endif


namespace MPCWalkgen{

	class RealClock {

		//
		// Public types
		//
	public:

		typedef unsigned long long IntegerType;

		//
		// Public methods:
		//
	public:
		RealClock();
		~RealClock();

		void ReserveMemory(
			int num_max_counters, 
			IntegerType max_computation_time	// Estimated boundary on the computation time [mus]
			);

		void GetFrequency(unsigned long long milliseconds);

		int StartCounter();

		/// \brief Stop counter associated with counter_index
		void StopCounter(int index);
		void StopLastCounter();

		/// \brief Give the latest measured time and decrease size of vector
		double PopBackTime();
		double GetTime(int counter);

		/// \brief Give the total measured time of last counter and decrease size of vector
		double PopBackTotalTime();
		double GetTotalTime(int counter);

		double GetMaxTime(int counter);

		inline int GetNumCounters() {return num_counters_;};
		inline int GetNumTotalCounters() {return static_cast<int>(total_ticks_vec_.size());};

		inline const std::vector<IntegerType> &ticks_distr_vec() const {return ticks_distr_vec_;};

		void ResetLocal();
		void ResetTotal();

		//
		// Private methods:
		//
	private:
#if (defined __LINUX__ || defined __VXWORKS__)
		IntegerType  __rdtsc( void );
#elif defined __WIN64__
#include 
#endif

		// 
		// Private data members:
		//
	private:
		int num_counters_;
		int num_max_counters_;

		IntegerType frequency_;
		std::vector<IntegerType> stop_tick_vec_;
		std::vector<IntegerType> start_tick_vec_;
		std::vector<IntegerType> total_ticks_vec_;	
		std::vector<IntegerType> max_ticks_vec_;
		
		std::vector<IntegerType> ticks_distr_vec_;

	};

}
#endif // MPC_WALKGEN_REALCLOCK_H
