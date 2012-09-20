#include <mpc-walkgen/types.h>


using namespace MPCWalkgen;

SelectionMatrices::SelectionMatrices(const MPCData &mpc_parameters)
:V(mpc_parameters.num_samples_horizon,mpc_parameters.num_samples_horizon)
,VT(mpc_parameters.num_samples_horizon,mpc_parameters.num_samples_horizon)
,VcX(mpc_parameters.num_samples_horizon)
,VcY(mpc_parameters.num_samples_horizon)
,Vf(mpc_parameters.num_samples_horizon,mpc_parameters.num_samples_horizon)
,VcfX(mpc_parameters.num_samples_horizon)
,VcfY(mpc_parameters.num_samples_horizon)
{}

void RelativeInequalities::resize(int rows, int cols){
  if (rows!=DX.rows() || cols!=DX.cols()){
    DX.setZero(rows, cols);
    DY.setZero(rows, cols);
    Dc.setZero(rows, cols);
  }else{
    DX.fill(0);
    DY.fill(0);
    Dc.fill(0);
  }

}