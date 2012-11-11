#ifndef DEBUG_H_
#define DEBUG_H_

#include <mpc-walkgen/types.h>

#include <vector>
#include <iostream>
#include <fstream>
#include <stdio.h>

class Debug {

public:
	// \brief Print of std::vector
	static void Cout(const char *name, const std::vector<double> &vec);
	static void Cout(const char *name, const std::vector<MPCWalkgen::CommonMatrixType> &vec);

	static void Cout(const char *name, const MPCWalkgen::CommonMatrixType &mat);

	static void WriteToFile(const char *filename, double value, const char *ending, const MPCWalkgen::CommonMatrixType &mat);
};

#endif /* DEBUG_H_ */
