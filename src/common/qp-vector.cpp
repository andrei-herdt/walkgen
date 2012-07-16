#include "qp-vector.h"
#include <Eigen/Cholesky>
#include <cmath>

using namespace MPCWalkgen;
using namespace Eigen;




QPVector::QPVector(const int nbrows)
:constantPart_(nbrows)
,vector_(nbrows)
,nbrows_(nbrows)
,rowOrder_(nbrows) {

  constantPart_.setZero();
  vector_.setZero();

  for (int i = 0; i < nbrows; ++i) {
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
  //nbrows_ = nbRows;
}

void QPVector::rowOrder(const Eigen::VectorXi &order){
  rowOrder_ = order;
}
