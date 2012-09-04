#include <mpc-walkgen/qp-vector.h>

#include <Eigen/Cholesky>
#include <cmath>

using namespace MPCWalkgen;
using namespace Eigen;




QPVector::QPVector(const int num_rows)
:constantPart_(num_rows)
,vector_(num_rows)
,num_rows_(num_rows)
,rowOrder_(num_rows) {

  constantPart_.setZero();
  vector_.setZero();

  for (int i = 0; i < num_rows; ++i) {
    rowOrder_(i) = i;
  }
}

QPVector::~QPVector(){}

void QPVector::addTerm(const VectorXd &vec, const int row) {
  int nbRows = vec.rows();
  for (int i = 0; i < nbRows; ++i){
    vector_(rowOrder_(row+i)) += vec(i);
  }
}

void QPVector::setTerm(const VectorXd &vec, const int row) {
  int nbRows = vec.rows();
  for (int i = 0; i < nbRows; ++i) {
    vector_(rowOrder_(row+i)) = vec(i);
  }
}

void QPVector::setConstantPart(const VectorXd &mat) {
  int nbRows = mat.rows();
  for (int i = 0; i <= nbRows; ++i) {
    constantPart_(rowOrder_(i)) = mat(i);
  }
}

void QPVector::reset(){
  vector_.fill(0);
}

void QPVector::resize(const int nbRows){
  //num_rows_ = nbRows;
}

void QPVector::rowOrder(const Eigen::VectorXi &order){
  rowOrder_ = order;
}
