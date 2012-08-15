
#include <cmath>
#include <cstdio>
#include <iostream>
#include <cstring>

#include "../src/humanoid/types.h"
#include "../src/common/qp-solver.h"

using namespace Eigen;
using namespace MPCWalkgen;


Eigen::VectorXd test_all_solvers(QPSolver &qp1, int nbvar, int nbcstr)
{
  qp1.reset();
  qp1.nbvar(nbvar);
  qp1.nbcstr(nbcstr);

  // create the qp problem
  MatrixXd Q(nbvar,nbvar);
  Q << 1,3,2,-4.6,-1.5  ,  3,2.5,6,4,8  ,  -5,-6,-4.2,-45,12  ,  -15,-12.87,0.025,0.154,1  ,  0,0,0,0,1;
  qp1.matrix(matrixQ).addTerm(Q.transpose()*Q);

  VectorXd P(nbvar);
  P << -2, -6, 0.5, -1.5, 8;
  qp1.vector(vectorP).addTerm(P);

  MatrixXd A(nbcstr,nbvar);
  A << 1,  1, 0.5, 0, 0,
    -1,  2, -2, -1.5, 0,
     2,  1, 0, 3, -4.5;
  qp1.matrix(matrixA).addTerm(A);

  VectorXd bl(nbcstr);
  bl << -4,0,-5;
  qp1.vector(vectorBL).addTerm(bl);

  VectorXd bu(nbcstr);
  bu << -2,2,3;
  qp1.vector(vectorBU).addTerm(bu);

  VectorXd xl(nbvar);
  xl.fill(-2);
  qp1.vector(vectorXL).addTerm(xl);

  VectorXd xu(nbvar);
  xu.fill(4);
  qp1.vector(vectorXU).addTerm(xu);

  MPCSolution result1;
  result1.reset();
  result1.initialSolution.resize(nbvar);
  result1.initialConstraints.resize(nbvar+nbcstr);

  qp1.solve(result1.qpSolution, result1.constraints,
     result1.initialSolution, result1.initialConstraints, true);

  return result1.qpSolution;
}


int main()
{
  bool success = true;

  int nbvar=5;
  int nbcstr=3;

  Eigen::VectorXd solution(nbvar);
  solution << 0.404315842674, -1.40431584267,       -2, 0.524701647986, 0.791078367425;

#ifdef MPC_WALKGEN_WITH_QPOASES
  std::cout << "bench-qpsolver test qpOASES " << std::endl;
  QPSolver * qp1 = createQPSolver(QPOASES, nbvar, nbcstr);
  Eigen::VectorXd qp1Solution = test_all_solvers(*qp1, nbvar, nbcstr);
  bool success1 = ((qp1Solution - solution).norm() < 1e-5);
  std::cout << "Solution QPOASES ("<< success1 <<"): " << qp1Solution.transpose() << std::endl;
  if (qp1) {
    delete qp1;
  }
  success = success1 && success;
#endif //MPC_WALKGEN_WITH_QPOASES

#ifdef MPC_WALKGEN_WITH_LSSOL
  std::cout << "bench-qpsolver test LSSOL " << std::endl;
  QPSolver * qp2 = createQPSolver(LSSOL, nbvar, nbcstr);
  Eigen::VectorXd qp2Solution = test_all_solvers(*qp2, nbvar, nbcstr);
  bool success2 = ((qp2Solution - solution).norm() < 1e-5);
  std::cout << "Solution LSSOL ("<< success2 <<") : " << qp2Solution.transpose() << std::endl;
  if (qp2) {
    delete qp2;
  }
  success = success2 && success;
#endif //MPC_WALKGEN_WITH_LSSOL

  return (success ? 0 : 1);
}
