#include <mpc-walkgen/qp-vector.h>

#include <Eigen/Cholesky>
#include <cmath>

using namespace MPCWalkgen;
using namespace Eigen;




QPVector::QPVector(const int num_rows)
:constantPart_(num_rows)
,vector_(num_rows)
,num_rows_(num_rows)
,index_order_(num_rows) {

  constantPart_.setZero();
  vector_.setZero();

  for (int i = 0; i < num_rows; ++i) {
    index_order_(i) = i;
  }
}

QPVector::~QPVector(){}

void QPVector::addTerm(const CommonVectorType &vec, const int first_row) {
  int num_rows = vec.rows();
  /*const double *vec_p = vec.data();
  const int *row_p = index_order_.data() + first_row;
  double *goal_vec_p = vector_.data();
  for (int i = 0; i < num_rows; ++i){
    *(goal_vec_p + *row_p) = *vec_p;
    ++row_p;
    ++vec_p;
  }*/

  // Non-optimized equivalent code
  for (int i = 0; i < num_rows; ++i){
    vector_(index_order_(first_row+i)) += vec(i);
  }
}

void QPVector::addTerm(double value, int first_row) {
  vector_(index_order_(first_row)) += value;
}

void QPVector::setConstantPart(const CommonVectorType &mat) {
  int nbRows = mat.rows();
  for (int i = 0; i <= nbRows; ++i) {
    constantPart_(index_order_(i)) = mat(i);
  }
}

void QPVector::reset(){
  vector_.fill(0);
}

void QPVector::resize(const int nbRows){
  //num_rows_ = nbRows;
}

void QPVector::rowOrder(const Eigen::VectorXi &order){
  index_order_ = order;
}
