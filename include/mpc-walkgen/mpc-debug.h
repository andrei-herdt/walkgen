#pragma once
#ifndef MPC_WALKGEN_MPC_DEBUG_H
#define MPC_WALKGEN_MPC_DEBUG_H

////////////////////////////////////////////////////////////////////////////////
///
///\file	mpc-debug.h
///\brief	A class to debug the program
///\author	Lafaye Jory
///\version	1.2
///\date	27/04/12
///
////////////////////////////////////////////////////////////////////////////////

#include <map>

namespace MPCWalkgen{

	enum TimeUnit{
		us,
		ms,
		s
	};

	class MPCDebug{
		public:
			MPCDebug(bool enable);
			~MPCDebug();

			void getTime(int id, bool start);
			double computeInterval(int id, TimeUnit unit = us);
			int nbIntervals(int id);

			void reset(int id);
			void reset();

		private:
			std::map<int,double> startTime_;
			std::map<int,double> endTime_;
			std::map<int,int> nbCount_;

			bool enable_;

	};

}

    //    //Variablen 
    //    LONGLONG g_Frequency, g_CurentCount, g_LastCount; 

    //    //Frequenz holen 
    //    if (!QueryPerformanceFrequency((LARGE_INTEGER*)&g_Frequency)) //
    //        std::cout << "Performance Counter nicht vorhanden" << std::endl; 

    //    //1. Messung 
    //    QueryPerformanceCounter((LARGE_INTEGER*)&g_CurentCount); 

    //2. Messung 
    //    QueryPerformanceCounter((LARGE_INTEGER*)&g_LastCount); 

    //    double dTimeDiff = (((double)(g_LastCount-g_CurentCount))/((double)g_Frequency));  

    //	std::cout << "Zeit: " << dTimeDiff << "at: " << time << std::endl; 


#endif // MPC_WALKGEN_MPC_DEBUG_H
