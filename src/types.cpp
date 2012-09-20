#include <mpc-walkgen/types.h>

using namespace MPCWalkgen;

SelectionMatrices::SelectionMatrices(int num_rows)
:V(num_rows, num_rows)
,VT(num_rows, num_rows)
,VcX(num_rows)
,VcY(num_rows)
,Vf(num_rows, num_rows)
,VcfX(num_rows)
,VcfY(num_rows)
,cm_feet_x(num_rows)
,cm_feet_y(num_rows)
,m_feet(num_rows, num_rows)
,m_feet_trans(num_rows, num_rows)
{}

void SelectionMatrices::SetZero() {
    V.setZero(); VT.setZero();
    VcX.setZero(); VcY.setZero();
    Vf.setZero();
    VcfX.setZero();  VcfY.setZero();
    cm_feet_x.setZero(); cm_feet_y.setZero();  
    m_feet.setZero(); m_feet_trans.setZero();
}

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