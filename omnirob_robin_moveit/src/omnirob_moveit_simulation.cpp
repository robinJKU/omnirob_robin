#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "omnirob_moveit");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* This sleep is ONLY to allow Rviz to come up */
  sleep(8.0);

  // .. _RobotModelLoader: http://docs.ros.org/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());


  // Using the :moveit_core:`RobotModel`, we can construct a
  // :moveit_core:`RobotState` that maintains the configuration
  // of the robot. We will set all joints in the state to their
  // default values. We can then get a
  // :moveit_core:`JointModelGroup`, which represents the robot
  // model for a particular group, e.g. the "right_arm" of the PR2
  // robot.
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("lwa");
  //robot_state::RobotState kinematic_state(kinematic_model);
  /*
  // get home srdf positions
  kinematic_state.setToDefaultValues(joint_model_group, "test_pose");
  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

 // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retreive the current set of joint values stored in the state
  std::vector<double> joint_values;
  kinematic_state.copyJointGroupPositions(joint_model_group, joint_values);
  */

  // defining the test_pose manually
  std::vector<double> joint_values (7);
  joint_values.at(0)= -0.3506;
  joint_values.at(1)= 0.9029;
  joint_values.at(2)= 0;
  joint_values.at(3)= 0.6166;
  joint_values.at(4)= 0;
  joint_values.at(5)= 1.1892;
  joint_values.at(6)= 1.5425;

  // The :move_group_interface:`MoveGroup` class can be easily 
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroup group_lwa("lwa");
  //group_lwa.setGoalTolerance(0.005);
  group_lwa.setPlanningTime(10); 
  group_lwa.setNumPlanningAttempts(2);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // (Optional) Create a publisher for visualizing plans in Rviz automatically.
  /*
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  */

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group_lwa.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group_lwa.getEndEffectorLink().c_str());


  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  group_lwa.setJointValueTarget(joint_values); // set target to test

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success =  group_lwa.plan(my_plan);

  ROS_INFO("Visualizing plan to test_pose (pose goal) %s",success?"":"FAILED"); 
  // Sleep to give Rviz time to visualize the plan. /
  //sleep(20.0);
  group_lwa.move(); // move default to test

  ROS_INFO("GoalJointTolerance: %f",group_lwa.getGoalJointTolerance());
  ROS_INFO("GoalOrientationTolerance: %f",group_lwa.getGoalOrientationTolerance());
  ROS_INFO("GoalPositionTolerance : %f",group_lwa.getGoalPositionTolerance()); 

  // ros::shutdown();  
  return 0;
}
