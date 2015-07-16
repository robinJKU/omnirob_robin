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

// tools
#include <omnirob_robin_tools_ros/ros_tools.h>

class pick_and_place_action_server
{
public:
	/**
	 * constructor
	 */
	pick_and_place_action_server( std::string pick_action_topic = "/pick_action"):
		lwa_(),
		pick_action_server_(node_handle_, pick_action_topic, false),
		blocker_("/unblock")
	{
		// wait for service
		ROS_INFO("Waiting for services");
		if( !ros::service::waitForService("/omnirob_robin/gripper/open_srv", 5) ||
		    !ros::service::waitForService("/omnirob_robin/gripper/close_srv", 5) )
		{
			ROS_ERROR("Didn't find all required services (gripper/open_srv, close_srv)");
		}else
		{
			open_gripper_client = node_handle_.serviceClient<omnirob_robin_msgs::move_gripper>("/omnirob_robin/gripper/open_srv");
			close_gripper_client = node_handle_.serviceClient<omnirob_robin_msgs::move_gripper>("/omnirob_robin/gripper/close_srv");
			ROS_INFO("Got all services");
		}

		// register callbacks
		pick_action_server_.registerGoalCallback(boost::bind(&pick_and_place_action_server::pick_and_place_goal_callback, this));
		pick_action_server_.registerPreemptCallback(boost::bind(&pick_and_place_action_server::pick_and_place_preempt_callback, this));

		// tf
		tf_listener_= new(tf::TransformListener);
		tf_broadcaster_ = new(tf::TransformBroadcaster);

		// define transformation between object frame and gripper frame
		tf::Transform to_object_rotated_from_gripper;
		to_object_rotated_from_gripper.setIdentity();
		float alpha = 0.0;
		float cos_alpha = cos(alpha);
		float sin_alpha = sin(alpha);
		tf::Matrix3x3 rotated = tf::Matrix3x3(0.0, -sin_alpha, cos_alpha,    0.0, cos_alpha, sin_alpha,   -1.0,0.0,0.0);
		to_object_rotated_from_gripper.setBasis( rotated );

		float shifted = 0.17; // distance between gripper and tcp
		tf::Vector3 shift_vec = tf::Vector3( 0.0, -0.04, -shifted );
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

		// start server
		ROS_INFO("Start server");
		pick_action_server_.start();
	}
	/**
	 * destructor
	 */
	~pick_and_place_action_server()
	{
		if( tf_listener_!=NULL)
			delete tf_listener_;

		if( tf_broadcaster_!=NULL)
			delete tf_broadcaster_;
	}

private:
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
		tf_broadcaster_->sendTransform( tf::StampedTransform( omnirob_geometry_tools::calc_tf( lwa_link_7_target_pose), ros::Time::now(), "base_link", "lwa/link_7/target_pose")); // todo: remove

		// plan path above pick pose
		moveit::planning_interface::MoveGroup::Plan plan_above_pick_pose, plan_pick_pose;
		moveit::planning_interface::MoveGroup::Plan plan_above_pick_pose_reversed, plan_pick_pose_reversed;

		geometry_msgs::Pose lwa_link_7_above_target_pose = geometry_msgs::Pose(lwa_link_7_target_pose);
		lwa_link_7_above_target_pose.position.z += 1e-1;

		tf_broadcaster_->sendTransform( tf::StampedTransform( omnirob_geometry_tools::calc_tf( lwa_link_7_above_target_pose), ros::Time::now(), "base_link", "lwa/link_7/target_pose_above")); // todo: remove

		ROS_INFO("Plan path to intermediate point");
		unsigned int ii=0;
		while(ii<5){
			moveit_msgs::Constraints constraints;
			constraints.joint_constraints.resize(1);
			constraints.joint_constraints[0].joint_name = "lwa/joint_3";
			constraints.joint_constraints[0].weight = 1.0;
			constraints.joint_constraints[0].position = 0.0;
			constraints.joint_constraints[0].tolerance_above = 0.1;
			constraints.joint_constraints[0].tolerance_below = 0.1;

			lwa_.plan_continuous_path_.lwa_move_group_->setPathConstraints( constraints);

			if( lwa_.plan_continuous_path_.plan_path_to_pose( plan_above_pick_pose, lwa_link_7_above_target_pose, "/base_link") ){
				ROS_INFO("Found valid path");
				break;
			}
			if( (!ros::ok()) || (!pick_action_server_.isActive()))
				break;
			ii++;
		}
		if( !ros::ok())
			return;

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
		blocker_.block( boost::bind( &pick_and_place_action_server::visualize_plan, this));

		// plan path to pick pose
		ROS_INFO("Plan path to pick pose");
		ii=0;
		std::vector<double> last_configuration = plan_above_pick_pose.trajectory_.joint_trajectory.points.back().positions;
		while(ii<5){
			moveit_msgs::Constraints constraints;
			constraints.joint_constraints.resize(1);
			constraints.joint_constraints[0].joint_name = "lwa/joint_3";
			constraints.joint_constraints[0].weight = 1.0;
			constraints.joint_constraints[0].position = 0.0;
			constraints.joint_constraints[0].tolerance_above = 0.1;
			constraints.joint_constraints[0].tolerance_below = 0.1;

			lwa_.plan_continuous_path_.lwa_move_group_->setPathConstraints( constraints);

			if( lwa_.plan_continuous_path_.plan_path_to_pose( plan_pick_pose, last_configuration, lwa_link_7_target_pose, "/base_link") ){
				ROS_INFO("Found valid path");
				break;
			}
			if( (!ros::ok) || (!pick_action_server_.isActive()))
				break;
			ii++;
		}
		if( !ros::ok())
			return;
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

		// todo: remove
		blocker_.block( boost::bind( &pick_and_place_action_server::visualize_plan, this));

		// reverse plans
		plan_above_pick_pose_reversed = moveit_tools::reverse_plan( plan_above_pick_pose);
		plan_pick_pose_reversed = moveit_tools::reverse_plan( plan_pick_pose);

		// open gripper
		omnirob_robin_msgs::move_gripper move_gripper_msgs;
		open_gripper_client.call( move_gripper_msgs);
		if( !move_gripper_msgs.response.error_message.empty() )
		{
			result.error_message = "Opening gripper failed";
			ROS_ERROR("%s", result.error_message.c_str());

			// cancel request
			pick_action_server_.setAborted( result);
			return;
		}

		// move to pick pose
		ROS_INFO("start first execution");
		std::string error_message = lwa_.execute_continuous_path_.execute_path_blocking( plan_above_pick_pose);
		if( !error_message.empty() ){
			result.error_message = "Execution of plan: above pick pose failed";
			ROS_ERROR("%s", result.error_message.c_str());

			// cancel request
			pick_action_server_.setAborted( result);
			return;
		}

		ROS_INFO("start second execution");
		error_message = lwa_.execute_continuous_path_.execute_path_blocking( plan_pick_pose);
		if( !error_message.empty() ){
			result.error_message = "Execution of plan: pick pose failed";
			ROS_ERROR("%s", result.error_message.c_str());

			// move home
			lwa_.execute_continuous_path_.execute_path_blocking( plan_above_pick_pose_reversed);

			// cancel request
			pick_action_server_.setAborted( result);
			return;
		}

		// close gripper
		close_gripper_client.call( move_gripper_msgs);
		if( !move_gripper_msgs.response.error_message.empty() )
		{
			result.error_message = "Closing  gripper failed";
			ROS_ERROR("%s", result.error_message.c_str());

			// move home
			lwa_.execute_continuous_path_.execute_path_blocking( plan_pick_pose_reversed);
			lwa_.execute_continuous_path_.execute_path_blocking( plan_above_pick_pose_reversed);

			// cancel request
			pick_action_server_.setAborted( result);
			return;
		}

		blocker_.block();

		// move home
		error_message = lwa_.execute_continuous_path_.execute_path_blocking( plan_pick_pose_reversed);
		if( !error_message.empty())
		{
			result.error_message = "Execution of plan: pick pose reversed failed";
			ROS_ERROR("%s", result.error_message.c_str());

			// move home
			lwa_.execute_continuous_path_.execute_path_blocking( plan_above_pick_pose_reversed);

			// cancel request
			pick_action_server_.setAborted( result);
			return;
		}

		error_message = lwa_.execute_continuous_path_.execute_path_blocking( plan_above_pick_pose_reversed);
		if( !error_message.empty())
		{
			result.error_message = "Execution of plan: above pick pose reversed failed";
			ROS_ERROR("%s", result.error_message.c_str());

			// cancel request
			pick_action_server_.setAborted( result);
			return;
		}

		// finish action
		pick_action_server_.setSucceeded( result);
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
		// correct the object yaw angle:
		//   for the following, only the projection onto the xy plane is considered
		//   the x-axis of the object frame should always look into the direction given by the vector from base_link to object_link
		double angle = atan2( to_base_link_from_object.getOrigin()[1], to_base_link_from_object.getOrigin()[0]); // angle between x axis and direction vector
		tf::Quaternion rotate_tangential;
		rotate_tangential.setRPY( 0.0, 0.0, angle);

		tf::Transform to_base_link_from_object_rotated = tf::Transform(to_base_link_from_object);
		to_base_link_from_object_rotated.setRotation( rotate_tangential);

		// to_base_link_from_object_rotated.getOrigin()[2] += 0e-2;

		// calculate grasp
		tf::Transform to_base_link_from_lwa_link7_desired = to_base_link_from_object_rotated*to_object_from_lwa_link7_;

		geometry_msgs::Pose lwa_link_7_target_pose;
		tf::poseTFToMsg(to_base_link_from_lwa_link7_desired, lwa_link_7_target_pose);
		return lwa_link_7_target_pose;
	}

public:
	void visualize_plan()
	{
		lwa_.plan_continuous_path_.visualize_last_plan();
	}

private:
	// node handle
	ros::NodeHandle node_handle_;

	// action servers
	actionlib::SimpleActionServer<omnirob_robin_moveit::pick_and_placeAction> pick_action_server_;

	// tf
	tf::TransformListener* tf_listener_;
	tf::TransformBroadcaster* tf_broadcaster_;
	tf::Transform to_object_from_lwa_link7_;

	// lwa control
	lwa_planner_and_executer lwa_;
	ros::ServiceClient open_gripper_client, close_gripper_client;

	// todo: remove
	omnirob_ros_tools::blocker blocker_;
};

int main(int argc, char **argv)
{

	// initialize ros node
	ros::init(argc, argv, "pick_and_place_server");
	
	ros::AsyncSpinner spinner(4); // required because both the server and the client run in the same node
	spinner.start();
	
	pick_and_place_action_server lwa_pick_and_place_server;

	ros::spin();
	return 0;

}// main
