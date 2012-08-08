/*
*  The purpose of this test is to simply check the installation
*  of lssol. Especially, we want to make sure that the link was succesful
*  The problem solved is hence quite simple:
*   min    || ( -1   -2) (x0)  - (-1) ||
*   x0,x1  || ( -2   -1) (x1)    ( 1) ||
*  The solution is (-1, 1)
*/

#include <cmath>
#include <cstdio>
#include <iostream>
#include <cstring>

#include "../src/common/qp-solver.h"
#include "../src/humanoid/types.h"
#include <mpc-walkgen/common/qp-solver-type.h>

using namespace Eigen;
using namespace MPCWalkgen;


/* Solve a simple quadratic programming problem: find values of x that minimize
f(x) = 1/2 x1^2 + x2^2 -x1.x2 - 2x1 - 6x2

subject to
 x1 +  x2 ≤ 2
–x1 + 2x2 ≤ 2
2x1 +  x2 ≤ 3
0 ≤ x1, 0 ≤ x2.

Expected solution:
x   = 0.6667, 1.3333
obj = -8.2222
*/

bool testQP (QPSolver & qp)
{
	qp.reset();
	qp.nbvar(2);
	qp.nbcstr(3);


	// create the qp problem
	Matrix2d Q;
	Q << 1, -1, -1, 2;
	Q *= 0.5;
	qp.matrix(matrixQ).addTerm(Q);

	Vector2d P;
	P << -2, -6;
	qp.vector(vectorP).addTerm(P);

	MatrixXd A(3,2);
	A << 1,  1,
	  -1,  2,
	   2,  1;
	qp.matrix(matrixA).addTerm(A);

	Vector3d bl;
	bl << -1e10, -1e10, -1e10;
	qp.vector(vectorBL).addTerm(bl);


	Vector3d bu;
	bu << 2,2,3;
	qp.vector(vectorBU).addTerm(bu);

	Vector2d xl;
	xl.fill(0);
	qp.vector(vectorXL).addTerm(xl);

	Vector2d xu;
	xu.fill(1e10);
	qp.vector(vectorXU).addTerm(xu);

	MPCSolution solution;
	solution.reset();
	solution.initialSolution.resize(2);
	solution.initialConstraints.resize(2+3);

	qp.solve(solution.qpSolution, solution.constraints,
		 solution.initialSolution, solution.initialConstraints,
		 true);

	Vector2d expectedResult;
	expectedResult << 2./3., 4./3.;
	std::cout << solution.qpSolution.transpose() << std::endl;
	bool success = ((solution.qpSolution - expectedResult).norm() < 1e-4);
	return success;
}


int main()
{
	bool success = true;
	QPSolver * solver = NULL;
#ifdef MPC_WALKGEN_WITH_QPOASES
	std::cout << "Testing qpOASES " << std::endl;
	solver = createQPSolver(QPSOLVERTYPE_QPOASES, 2 , 3);
	success = testQP(*solver) && success;
	if (solver) {
		delete solver;
	}
#endif //MPC_WALKGEN_WITH_QPOASES

#ifdef MPC_WALKGEN_WITH_LSSOL
	std::cout << "Testing LSSOL " << std::endl;
	solver = createQPSolver(QPSOLVERTYPE_LSSOL, 2 , 3);
	success = testQP(*solver) && success;
	if (solver) {
		delete solver;
	}
#endif //MPC_WALKGEN_WITH_LSSOL
	return (success ? 0 : 1);
}
