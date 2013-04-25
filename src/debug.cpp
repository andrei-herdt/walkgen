#include <mpc-walkgen/debug.h>

using namespace std;

void Debug::Print(const char *string) {
	printf(string, "\n");
}

void Debug::Print(const char *name, bool val) {
	printf("%s", name); printf(": ");
	val ? printf("true \n") : printf("false \n");
}

void Debug::Print(const char *name, int val) {
	printf("%s", name); printf(": ");
	printf("%d \n", val);
}

void Debug::Print(const char *name, double val) {
	printf(name, ": ");
	printf("%f \n", val);
}

void Debug::Cout(const char *name, bool val) {
	cout << name << ": " << endl;
	val ? cout << "true" << endl : cout << "false" << endl;
}

void Debug::Cout(const char *name, const vector<double> &vec) {
	cout << name << endl;
	for (unsigned i = 0; i < vec.size(); i++) {
		cout << vec[i] << " ";
	}
	cout << endl;
}

void Debug::Cout(const char *name, const vector<unsigned long long> &vec) {
	cout << name << endl;
	for (unsigned i=0; i < vec.size(); i++) {
		cout << vec[i] << " ";
	}
	cout << endl;
}

void Debug::Cout(const char *name, const vector<MPCWalkgen::CommonMatrixType> &vec) {
	cout << name << endl;
	for (unsigned i=0; i < vec.size(); i++) {
		cout << vec[i] << " " << endl;
	}
	cout << endl;
}

void Debug::Cout(const char *name, const MPCWalkgen::CommonMatrixType &mat) {
	cout << name << ": [" << mat.rows() << "," << mat.cols() << "]" << endl;
	cout << mat << endl;
}

void Debug::Cout(const char *name, const MPCWalkgen::CommonVectorType &vec) {
	cout << name << ": [" << vec.rows() << "]" << endl;
	cout << vec.transpose() << endl;
}

void Debug::WriteToFile(const char *filename, double value, const char *ending, const MPCWalkgen::CommonMatrixType &mat) {
	char file_index[256];
	sprintf(file_index, "%.4f", value);
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
