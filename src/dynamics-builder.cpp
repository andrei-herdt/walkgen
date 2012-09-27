#include <mpc-walkgen/dynamics-builder.h>
#include <mpc-walkgen/tools.h>

using namespace MPCWalkgen;

//
// Public methods:
//
DynamicsBuilder::DynamicsBuilder() {}

DynamicsBuilder::~DynamicsBuilder() {}

void DynamicsBuilder::Build(DynamicsOrder dynamics_order, LinearDynamics &dyn, double height, double sample_period_first, 
                            double sample_period_rest, int num_samples) 
{

  switch (dynamics_order){
    case SECOND_ORDER:
      BuildSecondOrder(dyn, height, sample_period_first, sample_period_rest, num_samples);
      break;
    case THIRD_ORDER:
      BuildThirdOrder(dyn, height, sample_period_first, sample_period_rest, num_samples);
      break;
  }
}

//
// Private methods:
//
void DynamicsBuilder::BuildSecondOrder(LinearDynamics &dyn, double height, double sample_period_first, double sample_period_rest, int num_samples) {
    BuildSecondOrder(dyn.pos, height, sample_period_first, sample_period_rest, num_samples, POSITION);
    BuildSecondOrder(dyn.vel, height, sample_period_first, sample_period_rest, num_samples, VELOCITY);
    BuildSecondOrder(dyn.acc, height, sample_period_first, sample_period_rest, num_samples, ACCELERATION);
    BuildSecondOrder(dyn.cop, height, sample_period_first, sample_period_rest, num_samples, COP);
}

void DynamicsBuilder::BuildThirdOrder(LinearDynamics &dyn, double height, double sample_period_first, double sample_period_rest, int num_samples) {
    BuildThirdOrder(dyn.pos, height, sample_period_first, sample_period_rest, num_samples, POSITION);
    BuildThirdOrder(dyn.vel, height, sample_period_first, sample_period_rest, num_samples, VELOCITY);
    BuildThirdOrder(dyn.acc, height, sample_period_first, sample_period_rest, num_samples, ACCELERATION);
    BuildThirdOrder(dyn.jerk, height, sample_period_first, sample_period_rest, num_samples, JERK);
    BuildThirdOrder(dyn.cop, height, sample_period_first, sample_period_rest, num_samples, COP);
}

void DynamicsBuilder::BuildThirdOrder(LinearDynamicsMatrices &dyn, double height,
                                      double sample_period_first, double sample_period_rest, int num_samples, Derivative derivative) 
{
  assert(height > 0.);
  assert(num_samples > 0.);  
  assert(sample_period_first > 0.);
  assert(sample_period_rest > 0.);

  const double kGravity = 9.81;

  int N = num_samples;
  double s = sample_period_first;
  double T = sample_period_rest;

  dyn.S.setZero(N, 3);
  dyn.U.setZero(N, N);
  dyn.UT.setZero(N, N);
  dyn.UInv.setZero(N, N);
  dyn.UInvT.setZero(N, N);

  switch (derivative){
    case POSITION:
      for (int row = 0; row < N; ++row) {
        dyn.S(row,0) = 1;
        dyn.S(row,1) = row * T + s;
        dyn.S(row,2) = s * s / 2 + row * T * s + row * row * T * T / 2;

        dyn.U(row,0) = dyn.UT(0,row) = s*s*s/6 + row*T*s*s/2 + s*(row*row*T*T/2 );
        for (int col=1; col<N; col++) {
          if (col <= row) {
            dyn.U(row,col) = dyn.UT(col,row) =T*T*T/6 + 3*(row-col)*T*T*T/6 + 3*(row-col)*(row-col) * T * T * T / 6;
          }
        }
      }
      break;

    case VELOCITY:
      for (int row=0;row<N;row++) {
        dyn.S(row,0) = 0.0;
        dyn.S(row,1) = 1.0;
        dyn.S(row,2) = row*T + s;

        dyn.U(row,0) = dyn.UT(0,row) = s*s/2 + row*T*s;
        for (int col=1; col<N; col++) {
          if (col<=row){
            dyn.U(row,col) = dyn.UT(col,row) = T*T/2 + (row-col)*T*T;
          }
        }
      }
      break;

    case ACCELERATION:
      for (int row=0; row<N; row++) {
        dyn.S(row,2) = 1.0;

        dyn.U(row,0) = dyn.UT(0,row) = s;
        for (int col=1; col<N; col++) {
          if (col<=row){
            dyn.U(row,col) = dyn.UT(col,row) = T;
          }
        }
      }
      break;

    case COP:
      for (int row=0; row<N; row++) {
        dyn.S(row,0) = 1;
        dyn.S(row,1) = row*T + s;
        dyn.S(row,2) = s*s/2 + row*T*s + row*row*T*T/2 - height / kGravity;

        dyn.U(row,0) = dyn.UT(0,row) =s*s*s/6 + row*T*s*s/2 + s*(row*row*T*T/2 - height / kGravity);
        for(int col=1; col<N; col++){
          if (col <= row) {
            dyn.U(row,col) = dyn.UT(col,row) = T*T*T/6 + 3*(row-col)*T*T*T/6 + 3*(row-col)*(row-col)*T*T*T/6 - T * height / kGravity;
          }
        }

      }
      inverse(dyn.U, dyn.UInv);
      dyn.UInvT=dyn.UInv.transpose();
      break;

    default:
      dyn.U.setIdentity();
      dyn.UT.setIdentity();
      break;
  }
}


void DynamicsBuilder::BuildSecondOrder(LinearDynamicsMatrices &dyn, double height,
                                       double sample_period_first, double sample_period_rest, 
                                       int num_samples, Derivative derivative) 
{
  assert(height > 0.);
  assert(num_samples > 0.);  
  assert(sample_period_first > 0.);
  assert(sample_period_rest > 0.);

  const double kGravity = 9.81;

  dyn.S.setZero(num_samples, 2);
  dyn.U.setZero(num_samples, num_samples);
  dyn.UT.setZero(num_samples, num_samples);
  dyn.UInv.setZero(num_samples, num_samples);
  dyn.UInvT.setZero(num_samples, num_samples);

  double sp1 = sample_period_first;
  double sp2 = sample_period_rest;
  double sp2sp2 = sp2 * sp2;
  double sp1sp1 = sp1 * sp1;
  switch (derivative){
    case POSITION:
      for (int row = 0; row < num_samples; ++row) {
        dyn.S(row, 0) = 1;
        dyn.S(row, 1) = row * sp2 + sp1;

        dyn.U(row, 0) = dyn.UT(0, row) = sp1sp1 / 2 + sp1 * row * sp2;
        for (int col = 1; col <= row; col++) {
          dyn.U(row, col) = dyn.UT(col, row) = sp2sp2 / 2 + (row - col) * sp2sp2;
        }
      }
      break;

    case VELOCITY:
      for (int row = 0; row < num_samples; row++) {
        dyn.S(row, 1) = 1.0;

        dyn.U(row, 0) = dyn.UT(0, row) = sp1;
        for (int col = 1; col <= row; col++) {
          dyn.U(row, col) = dyn.UT(col, row) = sp2;
        }
      }
      break;

    case ACCELERATION:
      dyn.U.setIdentity();
      dyn.UT.setIdentity();
      break;

    case COP:
      for (int row = 0; row < num_samples; ++row) {
        dyn.S(row, 0) = 1;
        dyn.S(row, 1) = row * sp2 + sp1;

        dyn.U(row, 0) = dyn.UT(0, row) = sp1sp1 / 2 + sp1 * row * sp2;
        for (int col = 1; col <= row; col++) {
          dyn.U(row, col) = dyn.UT(col, row) = sp2sp2 / 2 + (row - col) * sp2sp2;
        }

        dyn.U(row, row) = dyn.UT(row, row) -= height / kGravity;
      }
      inverse(dyn.U, dyn.UInv);
      dyn.UInvT = dyn.UInv.transpose();
      break;

    case JERK:
      break;

    default:
      assert(false);
  }
}
