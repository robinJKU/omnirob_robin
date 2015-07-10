// lwa
#include <omnirob_robin_moveit/lwa_planner_and_executer.h>

// transformation
#include <tf/tf.h>
#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>

// actions
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <omnirob_robin_moveit/pick_and_placeAction.h>

// services
#include <std_srvs/Empty.h>

// messsages
#include <omnirob_robin_msgs/move_gripper.h>

class pick_and_place_action_server
{
public:
	/**
	 * constructor
	 */
	pick_and_place_action_server( std::string pick_action_topic = "/pick_action"):
		lwa_(),
		pick_action_server_(node_handle_, pick_action_topic, false), // todo: was ist node_handle_, wo wird es initialisiert und warum braucht das der constructor?
		action_is_blocked_(true)
	{

		ros::NodeHandle node_handl;

		// wait for service
		ROS_INFO("Wait for services");
		if( !ros::service::waitForService("/omnirob_robin/gripper/open_srv", 5) ||
		    !ros::service::waitForService("/omnirob_robin/gripper/close_srv", 5) )
		{
			ROS_ERROR("Didn't find all required services (gripper/open_srv, close_srv)");
		}else
		{
			open_gripper_client = node_handl.serviceClient<omnirob_robin_msgs::move_gripper>("/omnirob_robin/gripper/open_srv");
			close_gripper_client = node_handl.serviceClient<omnirob_robin_msgs::move_gripper>("/omnirob_robin/gripper/close_srv");
			ROS_INFO("Got all services");
		}

		// register callbacks
		pick_action_server_.registerGoalCallback(boost::bind(&pick_and_place_action_server::pick_and_place_goal_callback, this));
		pick_action_server_.registerPreemptCallback(boost::bind(&pick_and_place_action_server::pick_and_place_preempt_callback, this));

		// broadcast service
		unblock_service_ = node_handl.advertiseService("unblock", &pick_and_place_action_server::unblock, this);

		// tf
		tf_listener_= new(tf::TransformListener);

		// define transformation between object frame and gripper frame
		tf::Transform to_object_rotated_from_gripper;
		to_object_rotated_from_gripper.setIdentity();
		float alpha = 0.0;
		float cos_alpha = cos(alpha);
		float sin_alpha = sin(alpha);
		tf::Matrix3x3 rotated = tf::Matrix3x3(0.0, -sin_alpha, cos_alpha,    0.0, cos_alpha, sin_alpha,   -1.0,0.0,0.0);
		to_object_rotated_from_gripper.setBasis( rotated );

		float shifted = 0.15; // distance between gripper and tcp
		tf::Vector3 shift_vec = tf::Vector3( 0.0, 0.0, -shifted );
		to_object_rotated_from_gripper.setOrigin( to_object_rotated_from_gripper*shift_vec );

		// define transformation between object frame and lwa link
		tf::StampedTransform to_gripper_from_lwa_link7_stamped;
		try {
		  tf_listener_->waitForTransform("gripper/center_link", "lwa/link_7", ros::Time(0), ros::Duration(5.0) );
		  tf_listener_->lookupTransform("gripper/center_link", "lwa/link_7", ros::Time(0), to_gripper_from_lwa_link7_stamped);
		}
		  catch(tf::TransformException &e){
		  ROS_ERROR("No Transform found from lwa/link_7 to gripper/center_link: %s . Can't plan pick and place paths", e.what());
		  return;
		}
		to_object_from_lwa_link7_ = to_object_rotated_from_gripper*to_gripper_from_lwa_link7_stamped;
		to_object_from_lwa_link7_ = to_object_rotated_from_gripper;

		ROS_INFO("broadcast transform");
		tf::TransformBroadcaster br;
		ros::Duration(1.0).sleep();
		ros::spinOnce();
		br.sendTransform(tf::StampedTransform(to_object_rotated_from_gripper.inverse(), ros::Time::now(), "lwa/link_7", "object_rotated_d")); // todo: remove
		ros::Duration(1.0).sleep();
		ros::spinOnce();

		// start server
		ROS_INFO("start server");
		pick_action_server_.start();
	}
	/**
	 * destructor
	 */
	~pick_and_place_action_server()
	{
		if( tf_listener_!=NULL)
			delete tf_listener_;
	}

private:
	bool unblock( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
	{
		action_is_blocked_=false;
		return true;
	}

	void pick_and_place_goal_callback( void)
	{
		ROS_INFO("got pick request");
		omnirob_robin_moveit::pick_and_placeResult result;

		// check input
		omnirob_robin_moveit::pick_and_placeGoalConstPtr goal = pick_action_server_.acceptNewGoal();

		std::string object_id = goal->object_id;
		shape_msgs::SolidPrimitive object_primitive = goal->object_primitive;

		if(object_id.empty())
		{
			result.error_message = "Invalid object_id";
			ROS_ERROR("%s", result.error_message.c_str());
			pick_action_server_.setAborted( result);
			return;
		}

		tf::StampedTransform to_base_link_from_object;
		try
		{
			tf_listener_->waitForTransform("base_link", object_id, ros::Time(0), ros::Duration(5.0) );
			tf_listener_->lookupTransform("base_link", object_id, ros::Time(0), to_base_link_from_object);
		}
		catch(tf::TransformException &e)
		{
			result.error_message = "No Transform found from base_link to object" + std::string(e.what());
			ROS_ERROR("%s", result.error_message.c_str());
			pick_action_server_.setAborted( result);
			return;
		}

		// plan grasp
		geometry_msgs::Pose lwa_link_7_target_pose = calc_grasp( to_base_link_from_object);

		// plan path above pick pose
		moveit::planning_interface::MoveGroup::Plan plan_above_pick_pose, plan_pick_pose;

		geometry_msgs::Pose lwa_link_7_above_target_pose = geometry_msgs::Pose(lwa_link_7_target_pose);
		lwa_link_7_above_target_pose.position.z += 1e-1;
		ROS_INFO("Plan path to intermediate point");
		unsigned int ii=0;
		while(ii<5){
			if( lwa_.plan_continuous_path_.plan_path_to_pose( plan_above_pick_pose, lwa_link_7_above_target_pose, "/base_link") ){
				ROS_INFO("Found valid path");
				break;
			}
			if( !pick_action_server_.isActive())
				break;
			ii++;
		}
		if( !pick_action_server_.isActive())
		{
			result.error_message = "Action canceled";
			ROS_ERROR("%s", result.error_message.c_str());
			pick_action_server_.setAborted( result);
			return;
		}
		if(ii==5)
		{
			result.error_message = "Can't find a valid path to the intermediate point";
			ROS_ERROR("%s", result.error_message.c_str());
			pick_action_server_.setAborted( result);
			return;
		}

		// blocking node todo: remove
		while( action_is_blocked_ && ros::ok() )
		{
			ros::Duration(2.0).sleep();
			lwa_.plan_continuous_path_.visualize_plan();
			ros::spinOnce();
		}
		action_is_blocked_ = true;

		// plan path to pick pose
		ROS_INFO("Plan path to pick pose");
		ii=0;
		std::vector<double> last_configuration = plan_above_pick_pose.trajectory_.joint_trajectory.points.back().positions;
		while(ii<5){
			if( lwa_.plan_continuous_path_.plan_path_to_pose( plan_pick_pose, last_configuration, lwa_link_7_target_pose, "/base_link") ){
				ROS_INFO("Found valid path");
				break;
			}
			if( !pick_action_server_.isActive())
				break;
			ii++;
		}
		if( !pick_action_server_.isActive())
		{
			result.error_message = "Action canceled";
			ROS_ERROR("%s", result.error_message.c_str());
			pick_action_server_.setAborted( result);
			return;
		}
		if(ii==5)
		{
			result.error_message = "Can't find a valid path to the intermediate point";
			ROS_ERROR("%s", result.error_message.c_str());
			pick_action_server_.setAborted( result);
			return;
		}

		// open gripper
		omnirob_robin_msgs::move_gripper move_gripper_msgs;
		open_gripper_client.call( move_gripper_msgs);
		if( !move_gripper_msgs.response.error_message.empty() )
		{
			result.error_message = "Opening gripper failed";
			ROS_ERROR("%s", result.error_message.c_str());
			pick_action_server_.setAborted( result);
			return;
		}

		// blocking node todo: remove
		while( action_is_blocked_ && ros::ok()  )
		{
			ros::Duration(2.0).sleep();
			lwa_.plan_continuous_path_.visualize_plan();
			ros::spinOnce();
		}
		action_is_blocked_ = true;

		// execute first path
		ROS_INFO("start first execution");
		std::string error_message = lwa_.execute_continuous_path_.execute_path_blocking( plan_above_pick_pose);
		if( !error_message.empty() ){
			// todo: apport request
		}

		// blocking node todo: remove
		while( action_is_blocked_ && ros::ok() )
		{
			ROS_INFO("blocking");
			ros::Duration(2.0).sleep();
			ros::spinOnce();
		}
		action_is_blocked_ = true;

		ROS_INFO("start second execution");
		moveit_tools::print_plan( plan_above_pick_pose);
		moveit_tools::print_plan( plan_pick_pose);

		// execute second path
		error_message = lwa_.execute_continuous_path_.execute_path_blocking( plan_pick_pose);
		if( !error_message.empty() ){
			// todo: move home
			// todo: apport request
		}
		

		// blocking node todo: remove
		while( action_is_blocked_ && ros::ok() )
		{
			ROS_INFO("blocking");
			ros::Duration(2.0).sleep();
			ros::spinOnce();
		}
		action_is_blocked_ = true;

		close_gripper_client.call( move_gripper_msgs);
		if( !move_gripper_msgs.response.error_message.empty() )
		{
			result.error_message = "Closing  gripper failed";
			ROS_ERROR("%s", result.error_message.c_str());
			pick_action_server_.setAborted( result);
			return;
		}

		ROS_INFO("original path - above pick pose");
		moveit_tools::print_plan( plan_pick_pose);
		moveit_tools::reverse_plan( plan_pick_pose);
		ROS_INFO("reversed path");
		moveit_tools::print_plan( plan_pick_pose);

		// execute second path reverse
		lwa_.execute_continuous_path_.execute_path_blocking( plan_pick_pose);

		// blocking node todo: remove
		while( action_is_blocked_ && ros::ok() );
		action_is_blocked_ = true;

		ROS_INFO("original path - above pick pose");
		moveit_tools::print_plan( plan_above_pick_pose);
		moveit_tools::reverse_plan( plan_above_pick_pose);
		ROS_INFO("reversed path");
		moveit_tools::print_plan( plan_above_pick_pose);

		// execute second path reverse
		lwa_.execute_continuous_path_.execute_path_blocking( plan_above_pick_pose);

	}
	void pick_and_place_preempt_callback( void)
	{
		ROS_ERROR("pick and place action is preempted!");
		pick_action_server_.setPreempted();
	}

private:
	/**
	 * Calculates a grasp for the specified object
	 * @param to_base_link_from_object_pose: Object pose described in base_link frame
	 * @return lwa/link_7 frame for the grasp pose describet in base_link frame
	 */
	geometry_msgs::Pose calc_grasp( const tf::Transform &to_base_link_from_object ){
		// correct the object yaw angle
		//   for the following, only the projection onto the xy plane is considered:
		//   the x-axis of the object frame should always look into the direction given by the vector from base_link to object_link
		double angle = atan2( to_base_link_from_object.getOrigin()[1], to_base_link_from_object.getOrigin()[0]); // angle between x axis and direction vector
		tf::Quaternion rotate_tangential;
		rotate_tangential.setRPY( 0.0, 0.0, angle);

		tf::Transform to_base_link_from_object_rotated = tf::Transform(to_base_link_from_object);
		to_base_link_from_object_rotated.setRotation( rotate_tangential);

	    // calculate grasp
	    tf::Transform to_base_link_from_lwa_link7_desired = to_base_link_from_object_rotated*to_object_from_lwa_link7_;

	    // visualize grasp
	    ROS_INFO("broadcast transform");
		static tf::TransformBroadcaster br;
		ros::Duration(1.0).sleep();
		ros::spinOnce();
		br.sendTransform(tf::StampedTransform(to_base_link_from_object_rotated, ros::Time::now(), "base_link", "object_rotated")); // todo: remove
		br.sendTransform(tf::StampedTransform(to_object_from_lwa_link7_, ros::Time::now(), "object_rotated", "lwa/link_7_desired")); // todo: remove
		ros::Duration(1.0).sleep();
		ros::spinOnce();



	  	geometry_msgs::Pose lwa_link_7_target_pose;
	  	tf::poseTFToMsg(to_base_link_from_lwa_link7_desired, lwa_link_7_target_pose);
	  	return lwa_link_7_target_pose;
	}

private:
	// node handle
	ros::NodeHandle node_handle_;

	// action servers
	actionlib::SimpleActionServer<omnirob_robin_moveit::pick_and_placeAction> pick_action_server_;
	bool action_is_blocked_;

	// tf
	tf::TransformListener* tf_listener_;
	tf::Transform to_object_from_lwa_link7_;

	// lwa control
	lwa_planner_and_executer lwa_;
	ros::ServiceServer unblock_service_;
	ros::ServiceClient open_gripper_client, close_gripper_client;
};


tf::TransformListener* pListener;
tf::Transform to_base_link_from_tcp;

//// Service
//bool plan_path(omnirob_robin_moveit::PlanPath::Request &req,
//               omnirob_robin_moveit::PlanPath::Response &resp) {
//
//	pListener = new(tf::TransformListener);
//	std::string objectString = "cylinder_orange0"; //req.tf_object_name;   /////////////////////////////////////////////////////
//   // string tf_object_name //PlanPath.srv   //////////////////////////////////////////////////////////////////////////////////77
//
//	tf::StampedTransform to_base_link_from_object;
//		try {
//		  pListener->waitForTransform("base_link", objectString, ros::Time(0), ros::Duration(5.0) );
//		  pListener->lookupTransform("base_link", objectString, ros::Time(0), to_base_link_from_object);
//		}
//		  catch(tf::TransformException e){
//		  ROS_ERROR("No Transform found from base_link to object");
//		  resp.success = false;
//		  return true;
//		}
//
//	tf::Transform to_base_link_from_object_rotated;
//	to_base_link_from_object_rotated = tf::Transform(to_base_link_from_object);
//	to_base_link_from_object_rotated.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));
//
//    static tf::TransformBroadcaster br;
//    br.sendTransform(tf::StampedTransform(to_base_link_from_object_rotated, ros::Time::now(), "base_link", "object_rotated"));
//
//	tf::Transform to_object_rotated_from_gripper;
//	to_object_rotated_from_gripper.setIdentity();
//	float alpha = M_PI/2.0;
//	float cos_alpha = cos(alpha);
//	float sin_alpha = sin(alpha);
//	tf::Matrix3x3 rotated = tf::Matrix3x3(0.0, -cos_alpha, sin_alpha,    0.0, -sin_alpha, -cos_alpha,   1.0,0.0,0.0);
// 	to_object_rotated_from_gripper.setBasis( rotated );
//
//
//	float shifted = 0.15; // distance between gripper and tcp
//	tf::Vector3 shift_vec = tf::Vector3( 0.0, 0.0, -shifted );
//	to_object_rotated_from_gripper.setOrigin( to_object_rotated_from_gripper*shift_vec );
//
//      br.sendTransform(tf::StampedTransform(to_object_rotated_from_gripper, ros::Time::now(), "object_rotated", "gripper"));
//
//	tf::StampedTransform to_gripper_from_tcp;
//	try {
//	  pListener->waitForTransform("gripper/center_link", "lwa/link_7", ros::Time(0), ros::Duration(5.0) );
//	  pListener->lookupTransform("gripper/center_link", "lwa/link_7", ros::Time(0), to_gripper_from_tcp);
//	}
//	  catch(tf::TransformException e){
//	  ROS_ERROR("No Transform found from tcp to gripper");
//	  resp.success = false;
//	  return true;
//	}
//
//        to_base_link_from_tcp = to_base_link_from_object_rotated*to_object_rotated_from_gripper;//*to_gripper_from_tcp
//
//      br.sendTransform(tf::StampedTransform(to_base_link_from_object_rotated*to_object_rotated_from_gripper, ros::Time::now(), "base_link", "gripper_center_link"));
//
//  	geometry_msgs::Pose target_pose;
//  	tf::poseTFToMsg(to_base_link_from_tcp, target_pose);
//
//
//
///*
//	// Anpassen der Quaternions für Ausrichten des Greifers
//	tf::Quaternion q;
//	//position: 0.2 -0.5 1.3
//  	//q.setRPY(0.0, 0.0, 0.0); // Greifer nach oben
//  	//q.setRPY(0.0, -1.5, 0.0); // Greifer seitlich Richtung Stange
//
//  	//position: 0.4 -0.5 1.3
//  	//q.setRPY(0.0, 1.5, 0.0); // Greifer seitlich weg von Stange
//
//  	//position: 0.4 -0.5 1.0
//  	//q.setRPY(0.0, -3.14, 0.0); // Greifer nach unten
//
//  	q.setRPY(0.0, 0.7, 0.0);
//  	tf::quaternionTFToMsg(q, req.target.orientation);
//*/
//
//	tf::poseTFToMsg(to_base_link_from_tcp, target_pose);
///*
//	std::vector<geomeii_msgs::Pose> waypoints;
//	geomeii_msgs::Pose target_above_pose = target_pose;
//	target_above_pose.position.x += -0.2;
//	target_above_pose.position.y += 0.0;
//	target_above_pose.position.z += 0.15;
//
//	waypoints.push_back(target_above_pose);
//	waypoints.push_back(target_pose);
//*/
//
//
//	// ################ Planning to a Pose goal ################
//	// set target to received geomeii_msgs::Pose
//	// group_lwa->setPoseTarget(target_pose);
//
//
//	moveit_msgs::RobotTrajectory trajectory;
//
//	int i;
//	bool success = false;
//	for(i=0; i<5; i++){
//		// Now, we call the planner to compute the plan and visualize it.
//		/*double fraction = group_lwa->computeCartesianPath(waypoints,
//							0.01, //eef_step
//							0.0, //jump_threshold
//							trajectory);*/
//		// success =  group_lwa->plan(*my_plan);
//		if(success) {
//			//my_plan->trajectory_ = trajectory;
//			//success = true;
//			break;
//		}
//	}
//
//
//	resp.success = success;
//	return true; // has to always to be true for service callback
//  }
//
//
//
//
//
//
//  // Service
//  bool plan_path_above(omnirob_robin_moveit::PlanPath::Request &req,
//	         omnirob_robin_moveit::PlanPath::Response &resp) {
//
//
//	pListener = new(tf::TransformListener);
//	std::string objectString = "cylinder_orange0"; //req.tf_object_name;   /////////////////////////////////////////////////////
//   // string tf_object_name //PlanPath.srv   //////////////////////////////////////////////////////////////////////////////////77
//
//	tf::StampedTransform to_base_link_from_object;
//		try {
//		  pListener->waitForTransform("base_link", objectString, ros::Time(0), ros::Duration(5.0) );
//		  pListener->lookupTransform("base_link", objectString, ros::Time(0), to_base_link_from_object);
//		}
//		  catch(tf::TransformException e){
//		  ROS_ERROR("No Transform found from base_link to object");
//		  resp.success = false;
//		  return true;
//		}
//
//	tf::Transform to_base_link_from_object_rotated;
//	to_base_link_from_object_rotated = tf::Transform(to_base_link_from_object);
//	to_base_link_from_object_rotated.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));
//
//     static tf::TransformBroadcaster br;
//      br.sendTransform(tf::StampedTransform(to_base_link_from_object_rotated, ros::Time::now(), "base_link", "object_rotated"));
//
//
//	tf::Transform to_object_rotated_from_gripper;
//	to_object_rotated_from_gripper.setIdentity();
//	float alpha = M_PI/2.0; //////
//	float cos_alpha = cos(alpha);
//	float sin_alpha = sin(alpha);
//	tf::Matrix3x3 rotated = tf::Matrix3x3(0.0, -cos_alpha, sin_alpha,    0.0, -sin_alpha, -cos_alpha,   1.0,0.0,0.0);
// 	to_object_rotated_from_gripper.setBasis( rotated );
//
//
//	float shifted = 0.15; // distance between gripper and tcp
//	tf::Vector3 shift_vec = tf::Vector3( 0.0, 0.0, -shifted );
//	to_object_rotated_from_gripper.setOrigin( to_object_rotated_from_gripper*shift_vec );
//
//      br.sendTransform(tf::StampedTransform(to_object_rotated_from_gripper, ros::Time::now(), "object_rotated", "gripper"));
//
//	tf::StampedTransform to_gripper_from_tcp;
//		try {
//		  pListener->waitForTransform("gripper/center_link", "lwa/link_7", ros::Time(0), ros::Duration(5.0) );
//		  pListener->lookupTransform("gripper/center_link", "lwa/link_7", ros::Time(0), to_gripper_from_tcp);
//
//
//		}
//		  catch(tf::TransformException e){
//		  ROS_ERROR("No Transform found from tcp to gripper");
//		  resp.success = false;
//		  return true;
//		}
//
//        to_base_link_from_tcp = to_base_link_from_object_rotated*to_object_rotated_from_gripper;//*to_gripper_from_tcp
//
//      br.sendTransform(tf::StampedTransform(to_base_link_from_object_rotated*to_object_rotated_from_gripper, ros::Time::now(), "base_link", "gripper_center_link"));
//
//
//
//	geometry_msgs::Pose target_pose;
//	tf::poseTFToMsg(to_base_link_from_tcp, target_pose);
//	target_pose.position.x += -0.05;
//	target_pose.position.y += 0.0;
//	target_pose.position.z += 0.25;
//
//	// ################ Planning to a Pose goal ################
//	// set target to received geomeii_msgs::Pose
//	//group_lwa->setPoseTarget(target_pose);
//
//	moveit_msgs::RobotTrajectory trajectory;
//
//	int i;
//	bool success = false;
//	for(i=0; i<5; i++){
//		//success =  group_lwa->plan(*my_plan);
//		if(success) {
//			//my_plan->trajectory_ = trajectory;
//			//success = true;
//			break;
//		}
//	}
//
//
//	resp.success = success;
//	return true; // has to always to be true for service callback
//  }
//
//
//
//
//
//
//
//
//
//
//
//  bool execute_path(omnirob_robin_moveit::ExecutePath::Request &req,
//	         omnirob_robin_moveit::ExecutePath::Response &resp) {
//	if(req.is_ready){
//  	  //group_lwa->move();
//	  // bool success = (bool)
//	  //group_lwa->execute(*my_plan);
//	}
//	resp.success = true;
//	return true;
//  }
//
//  // Service
//  bool plan_home(omnirob_robin_moveit::PlanHome::Request &req,
//	         omnirob_robin_moveit::PlanHome::Response &resp) {
//
//	std::vector<double> joint_values (7);
//	joint_values.at(0)= 0;
//	joint_values.at(1)= 0.1;
//	joint_values.at(2)= 0;
//	joint_values.at(3)= 0.15;
//	joint_values.at(4)= 0;
//	joint_values.at(5)= 0.15;
//	joint_values.at(6)=  M_PI/3.0;
//
//	// ################ Planning to a Pose goal ################
// 	// group_lwa->setJointValueTarget(joint_values); // set target to home
//
//	// Now, we call the planner to compute the plan and visualize it.
//	//bool success =  group_lwa->plan(*my_plan);
//
//	bool success;
//	resp.success = success;
//	return true; // has to always to be true for service callback
//  }
//
//
//  bool add_collision(omnirob_robin_moveit::AddCollisionObj::Request &req,
//	         omnirob_robin_moveit::AddCollisionObj::Response &resp) {
//
//	pListener = new(tf::TransformListener);
//
//	// First, we will define the collision object message.
//	moveit_msgs::CollisionObject collision_object;
//	// collision_object.header.frame_id =  group_lwa->getPlanningFrame();
//
//	// The id of the object is used to identify it. /
//	collision_object.id = req.primitive_id;
//
//	collision_object.primitives.push_back(req.primitive);
//
//	// get transformation from frame_id to primitive_id
//	tf::StampedTransform tf_primitive_pose;
//	try {
//		//ROS_INFO("######################################## frame_id %s", collision_object.header.frame_id.c_str());
//		//ROS_INFO("######################################## req.primitive_id %s", req.primitive_id.c_str());
//	  pListener->waitForTransform(collision_object.header.frame_id, req.primitive_id, ros::Time(0), ros::Duration(5.0) );
//	  pListener->lookupTransform(collision_object.header.frame_id, req.primitive_id, ros::Time(0), tf_primitive_pose);
//
//	}
//	  catch(tf::TransformException e){
//	  ROS_ERROR("No Transform found from tcp to gripper");
//	  resp.success = false;
//	  return true;
//	}
//	geometry_msgs::Pose primitive_pose;
//	tf::poseTFToMsg(tf_primitive_pose, primitive_pose);
//
//	collision_object.primitive_poses.push_back(primitive_pose);
//	collision_object.operation = collision_object.ADD;
//
//	std::vector<moveit_msgs::CollisionObject> collision_objects;
//	collision_objects.push_back(collision_object);
//
//
//	// Now, let's add the collision object into the world
//	ROS_INFO("Add an object into the world");
//	//planning_scene_interface->addCollisionObjects(collision_objects);
//
//	resp.success = true;
//	return true;
//  }

/**
 * Client for pick and place tasks
 */
class pick_and_place_executer{
	public:
	/**
	 * constructor
	 */
	pick_and_place_executer( const std::string &topic = "/pick_action"):
		pick_action_client_( topic, false)
	{
		// initialize state
		state_ = IDLE;

		// connect to server
		ROS_INFO("Waiting 10sec for action server (topic :%s)", topic.c_str());
		ROS_INFO("Waiting 10sec for action server (topic :%s)", topic.c_str());
		int cnt=0;
		ros::Rate rate1Hz(1.0);
		while(cnt<10.0){
			if(pick_action_client_.isServerConnected()){
				ROS_INFO("connected");
				break;
			}
			rate1Hz.sleep();
			ros::spinOnce();
			cnt++;
		}
		if( !pick_action_client_.isServerConnected())
		{
			ROS_ERROR("Server with topic %s is not connected", topic.c_str());
		}

		// initialize callback handle
		done_callback_handle_ = boost::bind( &pick_and_place_executer::done_callback, this, _1, _2);

	}
	/**
	 * destructor
	 */
	~pick_and_place_executer()
	{}

	public:// pick and place interface
		/**
		 * Pick an object which is specified by the object_id, object_primitive and the tf which corresponds to object_id (tf broadcast)
		 * @param object_id: object identifier
		 * @param object_primitive: shape of the object - used for collision avoidance
		 * @param blocking: Enable - true, Disable - false
		 * @param timeout: only used for blocking call
		 */
		std::string pick_object( std::string object_id, shape_msgs::SolidPrimitive object_primitive, bool blocking=true, ros::Duration timeout = ros::Duration(100.0)){
			std::string error_message;
			// check state
			if( state_!=IDLE ){
				error_message = "pick and place action is already started";
				ROS_ERROR("%s",error_message.c_str());
				return error_message;
			}

			omnirob_robin_moveit::pick_and_placeGoal goal;
			goal.object_id = object_id;
			goal.object_primitive = object_primitive;
			state_ = EXECUTING;

			if( blocking)
			{
				ROS_INFO("send goal and wait");
				pick_action_client_.sendGoalAndWait( goal, timeout);
				state_ = IDLE;
			}
			else
			{
				pick_action_client_.sendGoal( goal, done_callback_handle_);
			}
			return error_message;
		}

	void done_callback( const actionlib::SimpleClientGoalState& goal_state, const omnirob_robin_moveit::pick_and_placeResultConstPtr& result ){
		if( !result->error_message.empty()){
			ROS_ERROR("Error during pick or place action: %s    can't finish execution", result->error_message.c_str());
		}
		state_=IDLE;
	}

	public: // state monitoring
		enum State {IDLE, EXECUTING};
		/**
		 * @see State enumeration
		 */
		State get_state(){
			return state_;
		}

	private:
		// action client
		actionlib::SimpleActionClient<omnirob_robin_moveit::pick_and_placeAction> pick_action_client_;
		actionlib::SimpleActionClient<omnirob_robin_moveit::pick_and_placeAction>::SimpleDoneCallback done_callback_handle_;

		// state variables
		State state_;
};



int main(int argc, char **argv)
{

	// initialize ros node
	ros::init(argc, argv, "lwa_pick_and_place");

	ros::AsyncSpinner spinner(4); // required because both the server and the client run in the same node
	spinner.start();

	std::string table_id = "table";
	shape_msgs::SolidPrimitive table_primitive;
	table_primitive.type = table_primitive.BOX;
	table_primitive.dimensions.push_back(0.8);
	table_primitive.dimensions.push_back(0.8);
	table_primitive.dimensions.push_back(0.8);
	geometry_msgs::Pose table_pose;
	table_pose.position.x=1.0;
	table_pose.position.y=0.0;
	table_pose.position.z=0.4;
	table_pose.orientation.x=0.0;
	table_pose.orientation.y=0.0;
	table_pose.orientation.z=0.0;
	table_pose.orientation.w=1.0;
	std::string table_pose_frame = "/world";

	lwa_continuous_path_planner planner;
	ROS_INFO("add collision object");
	planner.add_static_object( table_id, table_primitive, table_pose, table_pose_frame);

	pick_and_place_action_server lwa_pick_and_place_server;
	pick_and_place_executer lwa_pick_and_place;


	std::string object_id = "cylinder_orange0";
	shape_msgs::SolidPrimitive object_primitive;
	object_primitive.type = object_primitive.CYLINDER;
	object_primitive.dimensions.push_back(2e-1); // height
	object_primitive.dimensions.push_back(4e-2); // radius

	ROS_INFO("give call");
	lwa_pick_and_place.pick_object( object_id, object_primitive);

	ros::spin();
	return 0;

}// main

