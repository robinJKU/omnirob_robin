#ifndef VARIANCE_COST_FUNCTION_H_
#define VARIANCE_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>
#include <math.h> 
#include <ros/ros.h>

//#include <costmap_2d/costmap_2d.h>
//#include <base_local_planner/map_grid.h>

namespace base_local_planner {

/**
 * when scoring a trajectory according to the values in mapgrid, we can take
 *return the value of the last point (if no of the earlier points were in
 * return collision), the sum for all points, or the product of all (non-zero) points
 */

/**
 * This class provides cost based on a map_grid of a small area of the world.
 * The map_grid covers a the costmap, the costmap containing the information
 * about sensed obstacles. The map_grid is used by setting
 * certain cells to distance 0, and then propagating distances around them,
 * filling up the area reachable around them.
 *
 * The approach using grid_maps is used for computational efficiency, allowing to
 * score hundreds of trajectories very quickly.
 *
 * This can be used to favor trajectories which stay on a given path, or which
 * approach a given goal.
 * @param costmap_ros Reference to object giving updates of obstacles around robot
 * @param xshift where the scoring point is with respect to robot center pose
 * @param yshift where the scoring point is with respect to robot center pose
 * @param is_local_goal_function, scores for local goal rather than whole path
 * @param aggregationType how to combine costs along trajectory
 */
class VarianceCostFunction: public base_local_planner::TrajectoryCostFunction {
public:
  VarianceCostFunction(){

  if (ros::param::get("/move_base/DWAPlannerROS/weight_variance_x", weight_variance_x)==false){

      weight_variance_x=1.0;
  }

  if (ros::param::get("/move_base/DWAPlannerROS/weight_variance_y", weight_variance_y)==false){

      weight_variance_y=1.0;
  }

  if (ros::param::get("/move_base/DWAPlannerROS/weight_variance_th", weight_variance_th)==false){

      weight_variance_th=1.0;
  }

  };

  ~VarianceCostFunction() {}

  double scoreTrajectory(Trajectory &traj);
  bool prepare();

private:

  double weight_variance_x,weight_variance_y,weight_variance_th;

};

} /* namespace base_local_planner */
#endif /* VARIANCE_COST_FUNCTION_H_ */
