#include <omnirob_robin_dwa_local_planner/variance_cost_function.h>
#include <iostream>

namespace base_local_planner {

bool VarianceCostFunction::prepare() {
  return true;
}

double VarianceCostFunction::scoreTrajectory(Trajectory &traj) {
  double cost = 0.0;
  double deltaxR,deltayR,deltathR;

  int num=traj.getPointsSize()-1;

  deltaxR = traj.xv_*traj.time_delta_*num;
  deltayR = traj.yv_*traj.time_delta_*num;
  deltathR= traj.thetav_*traj.time_delta_*num;

  double varx, vary, varth;
   
  varx=0.00247*0.00247+0.06562*0.06562*deltayR*deltayR+0.00157*0.00157*deltathR*deltathR;
  vary=0.00096*0.00096*deltaxR*deltaxR+0.05174*0.05174*deltayR*deltayR+0.00133*0.00133*deltathR*deltathR;
  varth=0.00195*0.00195+0.00879*0.00879*deltayR*deltayR+0.00215*0.00215*deltathR*deltathR;

  cost=weight_variance_x*varx+weight_variance_y*vary+weight_variance_th*varth;
  
  return cost;
}

} /* namespace base_local_planner */

