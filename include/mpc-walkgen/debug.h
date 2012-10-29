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
	static void Cout(const char *name, std::vector<double> vec);

	static void WriteToDatFile(const char *filename, double value, const MPCWalkgen::CommonMatrixType &mat);
};

#endif /* DEBUG_H_ */