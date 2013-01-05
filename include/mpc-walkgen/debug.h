#ifndef DEBUG_H_
#define DEBUG_H_

#ifdef MATLAB_MEX_FILE
#include "mex.h"
#define printf mexPrintf
#endif

#include <mpc-walkgen/types.h>

#include <vector>
#include <iostream>
#include <fstream>
#include <stdio.h>

class Debug {

public:

	static void Cout(const char *name, double val);

	// \brief Print of std::vector
	static void Cout(const char *name, const std::vector<double> &vec);
	static void Cout(const char *name, const std::vector<MPCWalkgen::CommonMatrixType> &vec);

	static void Cout(const char *name, const MPCWalkgen::CommonMatrixType &mat);
	static void Cout(const char *name, const MPCWalkgen::CommonVectorType &vec);

	static void WriteToFile(const char *filename, double value, const char *ending, const MPCWalkgen::CommonMatrixType &mat);
};

#endif /* DEBUG_H_ */
