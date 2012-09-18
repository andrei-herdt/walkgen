#include <mpc-walkgen/types.h>


using namespace MPCWalkgen;

SelectionMatrices::SelectionMatrices(const MPCData &mpc_parameters)
:V(mpc_parameters.nbsamples_qp,mpc_parameters.nbsamples_qp)
,VT(mpc_parameters.nbsamples_qp,mpc_parameters.nbsamples_qp)
,VcX(mpc_parameters.nbsamples_qp)
,VcY(mpc_parameters.nbsamples_qp)
,Vf(mpc_parameters.nbsamples_qp,mpc_parameters.nbsamples_qp)
,VcfX(mpc_parameters.nbsamples_qp)
,VcfY(mpc_parameters.nbsamples_qp)
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