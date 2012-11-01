#include <mpc-walkgen/debug.h>

using namespace std;

void Debug::Cout(const char *name, vector<double> vec) {
		cout << name << endl;
		for (int i=0; i < vec.size(); i++) {
			cout << vec[i] << " ";
		}
		cout << endl;
}

void Debug::WriteToFile(const char *filename, double value, const char *ending, const MPCWalkgen::CommonMatrixType &mat) {
	char file_index[256];
	sprintf(file_index, "%.2f", value);
	char name[256];
	strcpy(name, filename); // copy string one into the result.
	strcat(name, file_index);
	strcat(name, ".");
	strcat(name, ending);

	std::ofstream file(name);
	if (file.is_open()) {
		file << mat;
	}
}
