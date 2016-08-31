#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

#include "ros/ros.h"
#include "omnirob_robin_moveit/PlanPath.h"
#include "omnirob_robin_moveit/PlanHome.h"
#include "omnirob_robin_moveit/ExecutePath.h"
#include "omnirob_robin_moveit/AddCollisionObj.h"
#include <tf/tf.h>
#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>

 
  //allocates memory for the Class type pointer
  moveit::planning_interface::MoveGroup* group_lwa; 
  ros::NodeHandle *node_handle; 
  robot_state::RobotState *kinematic_state;
  moveit::planning_interface::MoveGroup::Plan *my_plan;

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;

  tf::TransformListener* pListener;
  tf::Transform to_base_link_from_tcp;



  // Service
  bool plan_path(omnirob_robin_moveit::PlanPath::Request &req,
	         omnirob_robin_moveit::PlanPath::Response &resp) {
		

	pListener = new(tf::TransformListener);
	std::string objectString = "cylinder_orange0"; //req.tf_object_name;   /////////////////////////////////////////////////////
   // string tf_object_name //PlanPath.srv   //////////////////////////////////////////////////////////////////////////////////77

	tf::StampedTransform to_base_link_from_object;
		try {            
		  pListener->waitForTransform("base_link", objectString, ros::Time(0), ros::Duration(5.0) );
		  pListener->lookupTransform("base_link", objectString, ros::Time(0), to_base_link_from_object);
		}
		  catch(tf::TransformException e){
		  ROS_ERROR("No Transform found from base_link to object");
		  resp.success = false;
		  return true;
		}

	tf::Transform to_base_link_from_object_rotated;
	to_base_link_from_object_rotated = tf::Transform(to_base_link_from_object);
	to_base_link_from_object_rotated.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));

     static tf::TransformBroadcaster br;
      br.sendTransform(tf::StampedTransform(to_base_link_from_object_rotated, ros::Time::now(), "base_link", "object_rotated"));


	tf::Transform to_object_rotated_from_gripper;
	to_object_rotated_from_gripper.setIdentity();
	float alpha = M_PI/2.0; //////
	float cos_alpha = cos(alpha);
	float sin_alpha = sin(alpha);
	tf::Matrix3x3 rotated = tf::Matrix3x3(0.0, -cos_alpha, sin_alpha,    0.0, -sin_alpha, -cos_alpha,   1.0,0.0,0.0);
 	to_object_rotated_from_gripper.setBasis( rotated );


	float shifted = 0.15; // distance between gripper and tcp 
	tf::Vector3 shift_vec = tf::Vector3( 0.0, 0.0, -shifted );
	to_object_rotated_from_gripper.setOrigin( to_object_rotated_from_gripper*shift_vec );	

      br.sendTransform(tf::StampedTransform(to_object_rotated_from_gripper, ros::Time::now(), "object_rotated", "gripper"));

	tf::StampedTransform to_gripper_from_tcp;
		try {            
		  pListener->waitForTransform("gripper/center_link", "lwa/link_7", ros::Time(0), ros::Duration(5.0) );
		  pListener->lookupTransform("gripper/center_link", "lwa/link_7", ros::Time(0), to_gripper_from_tcp);

		 
		}
		  catch(tf::TransformException e){
		  ROS_ERROR("No Transform found from tcp to gripper");
		  resp.success = false;
		  return true;
		}

        to_base_link_from_tcp = to_base_link_from_object_rotated*to_object_rotated_from_gripper;//*to_gripper_from_tcp

      br.sendTransform(tf::StampedTransform(to_base_link_from_object_rotated*to_object_rotated_from_gripper, ros::Time::now(), "base_link", "gripper_center_link"));

 	 



/*
	// Anpassen der Quaternions für Ausrichten des Greifers
	tf::Quaternion q;
	//position: 0.2 -0.5 1.3 
  	//q.setRPY(0.0, 0.0, 0.0); // Greifer nach oben
  	//q.setRPY(0.0, -1.5, 0.0); // Greifer seitlich Richtung Stange
  
  	//position: 0.4 -0.5 1.3 
  	//q.setRPY(0.0, 1.5, 0.0); // Greifer seitlich weg von Stange
  
  	//position: 0.4 -0.5 1.0
  	//q.setRPY(0.0, -3.14, 0.0); // Greifer nach unten

  	q.setRPY(0.0, 0.7, 0.0);
  	tf::quaternionTFToMsg(q, req.target.orientation);
*/
	
	geometry_msgs::Pose target_pose;
	tf::poseTFToMsg(to_base_link_from_tcp, target_pose);
/*
	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose target_above_pose = target_pose;
	target_above_pose.position.x += -0.2;
	target_above_pose.position.y += 0.0;
	target_above_pose.position.z += 0.15;

	waypoints.push_back(target_above_pose);
	waypoints.push_back(target_pose);
*/


	// ################ Planning to a Pose goal ################
	// set target to received geometry_msgs::Pose
	group_lwa->setPoseTarget(target_pose);


	moveit_msgs::RobotTrajectory trajectory;

	int i;
	bool success = false;
	for(i=0; i<5; i++){
		// Now, we call the planner to compute the plan and visualize it.
		/*double fraction = group_lwa->computeCartesianPath(waypoints, 
							0.01, //eef_step
							0.0, //jump_threshold
							trajectory);*/
		success =  group_lwa->plan(*my_plan);   
		if(success) { 
			//my_plan->trajectory_ = trajectory;
			//success = true;
			break;
		}
	}


	resp.success = success;
	return true; // has to always to be true for service callback
  }






  // Service
  bool plan_path_above(omnirob_robin_moveit::PlanPath::Request &req,
	         omnirob_robin_moveit::PlanPath::Response &resp) {
		

	pListener = new(tf::TransformListener);
	std::string objectString = "cylinder_orange0"; //req.tf_object_name;   /////////////////////////////////////////////////////
   // string tf_object_name //PlanPath.srv   //////////////////////////////////////////////////////////////////////////////////77

	tf::StampedTransform to_base_link_from_object;
		try {            
		  pListener->waitForTransform("base_link", objectString, ros::Time(0), ros::Duration(5.0) );
		  pListener->lookupTransform("base_link", objectString, ros::Time(0), to_base_link_from_object);
		}
		  catch(tf::TransformException e){
		  ROS_ERROR("No Transform found from base_link to object");
		  resp.success = false;
		  return true;
		}

	tf::Transform to_base_link_from_object_rotated;
	to_base_link_from_object_rotated = tf::Transform(to_base_link_from_object);
	to_base_link_from_object_rotated.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));

     static tf::TransformBroadcaster br;
      br.sendTransform(tf::StampedTransform(to_base_link_from_object_rotated, ros::Time::now(), "base_link", "object_rotated"));


	tf::Transform to_object_rotated_from_gripper;
	to_object_rotated_from_gripper.setIdentity();
	float alpha = M_PI/2.0; //////
	float cos_alpha = cos(alpha);
	float sin_alpha = sin(alpha);
	tf::Matrix3x3 rotated = tf::Matrix3x3(0.0, -cos_alpha, sin_alpha,    0.0, -sin_alpha, -cos_alpha,   1.0,0.0,0.0);
 	to_object_rotated_from_gripper.setBasis( rotated );


	float shifted = 0.15; // distance between gripper and tcp 
	tf::Vector3 shift_vec = tf::Vector3( 0.0, 0.0, -shifted );
	to_object_rotated_from_gripper.setOrigin( to_object_rotated_from_gripper*shift_vec );	

      br.sendTransform(tf::StampedTransform(to_object_rotated_from_gripper, ros::Time::now(), "object_rotated", "gripper"));

	tf::StampedTransform to_gripper_from_tcp;
		try {            
		  pListener->waitForTransform("gripper/center_link", "lwa/link_7", ros::Time(0), ros::Duration(5.0) );
		  pListener->lookupTransform("gripper/center_link", "lwa/link_7", ros::Time(0), to_gripper_from_tcp);

		 
		}
		  catch(tf::TransformException e){
		  ROS_ERROR("No Transform found from tcp to gripper");
		  resp.success = false;
		  return true;
		}

        to_base_link_from_tcp = to_base_link_from_object_rotated*to_object_rotated_from_gripper;//*to_gripper_from_tcp

      br.sendTransform(tf::StampedTransform(to_base_link_from_object_rotated*to_object_rotated_from_gripper, ros::Time::now(), "base_link", "gripper_center_link"));


	
	geometry_msgs::Pose target_pose;
	tf::poseTFToMsg(to_base_link_from_tcp, target_pose);
	target_pose.position.x += -0.05;
	target_pose.position.y += 0.0;
	target_pose.position.z += 0.25;

	// ################ Planning to a Pose goal ################
	// set target to received geometry_msgs::Pose
	group_lwa->setPoseTarget(target_pose);

	moveit_msgs::RobotTrajectory trajectory;

	int i;
	bool success = false;
	for(i=0; i<5; i++){
		success =  group_lwa->plan(*my_plan);   
		if(success) { 
			//my_plan->trajectory_ = trajectory;
			//success = true;
			break;
		}
	}


	resp.success = success;
	return true; // has to always to be true for service callback
  }











  bool execute_path(omnirob_robin_moveit::ExecutePath::Request &req,
	         omnirob_robin_moveit::ExecutePath::Response &resp) {
	if(req.is_ready){
  	  //group_lwa->move();
	  // bool success = (bool)
	  group_lwa->execute(*my_plan);
	}
	resp.success = true;
	return true;
  }

  // Service
  bool plan_home(omnirob_robin_moveit::PlanHome::Request &req,
	         omnirob_robin_moveit::PlanHome::Response &resp) {

	std::vector<double> joint_values (7);
	joint_values.at(0)= 0;
	joint_values.at(1)= 0.1;
	joint_values.at(2)= 0;
	joint_values.at(3)= 0.15;
	joint_values.at(4)= 0;
	joint_values.at(5)= 0.15;
	joint_values.at(6)=  M_PI/3.0;

	// ################ Planning to a Pose goal ################
 	group_lwa->setJointValueTarget(joint_values); // set target to home

	// Now, we call the planner to compute the plan and visualize it.
	bool success =  group_lwa->plan(*my_plan);

	resp.success = success;
	return true; // has to always to be true for service callback
  }


  bool add_collision(omnirob_robin_moveit::AddCollisionObj::Request &req,
	         omnirob_robin_moveit::AddCollisionObj::Response &resp) {

	pListener = new(tf::TransformListener);
  	  
	// First, we will define the collision object message.
	moveit_msgs::CollisionObject collision_object;
	collision_object.header.frame_id =  group_lwa->getPlanningFrame();

	// The id of the object is used to identify it. /
	collision_object.id = req.primitive_id;

	collision_object.primitives.push_back(req.primitive);

	// get transformation from frame_id to primitive_id
	tf::StampedTransform tf_primitive_pose;
	try {            
		//ROS_INFO("######################################## frame_id %s", collision_object.header.frame_id.c_str());	
		//ROS_INFO("######################################## req.primitive_id %s", req.primitive_id.c_str()); 
	  pListener->waitForTransform(collision_object.header.frame_id, req.primitive_id, ros::Time(0), ros::Duration(5.0) );
	  pListener->lookupTransform(collision_object.header.frame_id, req.primitive_id, ros::Time(0), tf_primitive_pose);

	}
	  catch(tf::TransformException e){
	  ROS_ERROR("No Transform found from tcp to gripper");
	  resp.success = false;
	  return true;
	}
	geometry_msgs::Pose primitive_pose;
	tf::poseTFToMsg(tf_primitive_pose, primitive_pose);

	collision_object.primitive_poses.push_back(primitive_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;  
	collision_objects.push_back(collision_object);


	// Now, let's add the collision object into the world
	ROS_INFO("Add an object into the world");  
	planning_scene_interface->addCollisionObjects(collision_objects);

	resp.success = true;
	return true;
  }



int main(int argc, char **argv)
{
  ros::init(argc, argv, "omnirob_moveit");
  ros::NodeHandle nh;
  node_handle = &nh;
  //ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // allow programs come up 
  sleep(4.0);


  // load robot model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  //ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());


  //  construct a RobotState` that maintains the configuration
  // of the robot. We will set all joints in the state to their
  // default values. We can then get a
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("lwa");
	robot_state::RobotState kinematic_state_init(kinematic_model);
  	kinematic_state = &kinematic_state_init;


  // get home srdf positions
  //kinematic_state->setToDefaultValues(joint_model_group, "test_pose");
  //const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

 // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retreive the current set of joint values stored in the state
  //std::vector<double> joint_values;
  //kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  //


  // setup a MoveGroup
  moveit::planning_interface::MoveGroup group("lwa"); //allocates memory for the group object
  group_lwa = &group; //group_lwa pointer points to the momory address of group object
  //group_lwa->setGoalTolerance(0.005);
  group_lwa->setPlanningTime(10); 
  group_lwa->setNumPlanningAttempts(2);


 // rostopic echo /omnirob_robin/joint_states 

  //Specify which reference frame to assume for poses specified without a reference frame. 
  group_lwa->setPoseReferenceFrame ("/base_link");

/*
  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 
*/

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_obj;
  
  planning_scene_interface = &planning_scene_interface_obj;

  //to be declared only once and not by every plan_path call	
  moveit::planning_interface::MoveGroup::Plan my_plan2;
  my_plan = &my_plan2;  

  // (Optional) Create a publisher for visualizing plans in Rviz automatically.
  ros::Publisher display_publisher = node_handle->advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  //moveit_msgs::DisplayTrajectory display_trajectory;




	  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("gripper/palm_link");
	  /* Print end-effector pose. Remember that this is in the model frame */
	  //ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
	  //ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
	  //Eigen::Vector3d translation = end_effector_state.translation();
	  //ROS_INFO("Positions: %f %f %f", translation[0], translation[1], translation[2]);



  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Planning Reference frame: %s", group_lwa->getPlanningFrame().c_str());
  ROS_INFO("Pose Reference frame: %s", group_lwa->getPoseReferenceFrame().c_str());


  // We can also print the name of the end-effector link for this group.
  //ROS_INFO("Reference frame: %s", group_lwa->getEndEffectorLink().c_str());

  
  //------------ Advertise service-----------------
  ros::ServiceServer service_plan = node_handle->advertiseService("PlanPath", plan_path);// /omnirob_robin_moveit/
  ros::ServiceServer service_plan_above = node_handle->advertiseService("PlanPathAbove", plan_path_above);// /omnirob_robin_moveit/

  ros::ServiceServer service_exec = node_handle->advertiseService("ExecutePath", execute_path);

  ros::ServiceServer service_home = node_handle->advertiseService("PlanHome", plan_home);

  ros::ServiceServer service_add_collision = node_handle->advertiseService("AddCollisionObj", add_collision);



  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setIdentity();
  transform.setOrigin(  tf::Vector3(1.1, 0.1, 0.9) );

  tf::Transform transform_box;
  transform_box.setIdentity();
  transform_box.setOrigin(  tf::Vector3(1.4, 0, 0.4) );

  // PUBLISH the robot state to topic /display_robot_state
  ros::Publisher robot_state_publisher = node_handle->advertise<moveit_msgs::DisplayRobotState>( "display_robot_state", 1 );

  // loop at 1 Hz //
  ros::Rate loop_rate(1); //????????????????????????????
  // get a robot state message describing the pose in kinematic_state //
  moveit_msgs::DisplayRobotState msg;
  while (true)
  {
    robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);

    // send the message to the RobotState display //
    robot_state_publisher.publish( msg );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "cylinder_orange0")); // luke !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    br.sendTransform(tf::StampedTransform(transform_box, ros::Time::now(), "odom", "box0")); 
    br.sendTransform(tf::StampedTransform(to_base_link_from_tcp, ros::Time::now(), "base_link", "tcp"));

    //let ROS send the message, then wait a while //
    ros::spinOnce();
    loop_rate.sleep();
  }

  //ros::spin(); 
  return 0;
}

