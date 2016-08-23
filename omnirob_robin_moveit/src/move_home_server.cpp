// lwa
#include <omnirob_robin_moveit/lwa_planner_and_executer.h>

// transformation
#include <tf/tf.h>
#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>

// actions
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <omnirob_robin_moveit/move_homeAction.h>

// services
#include <std_srvs/Empty.h>
#include <ros_common_robin_msgs/execute.h>

// messsages
#include <omnirob_robin_msgs/move_gripper.h>

// tools
#include <omnirob_robin_tools_ros/ros_tools.h>
#include <moveit/rdf_loader/rdf_loader.h>

class move_home_server
{
public:
	/**
	 * constructor
	 */
	move_home_server( std::string move_home_topic = "/move_home"):
		lwa_(),
		move_home_action_server_(node_handle_, move_home_topic + "_action", false),
		home_configuration_(7,0.0)
	{
		init( move_home_topic);
	}
	move_home_server( const std::vector<double> &home_configuration, std::string move_home_topic = "/move_home"):
		lwa_(),
		move_home_action_server_(node_handle_, move_home_topic + "_action", false)
	{
		home_configuration_ = home_configuration;
		if( home_configuration_.size()!=7)
		{
			ROS_WARN("Unexpected number of elements in home configuration. Got %u expected 7. Used default values (0,0,0,0,0,0,0) instead.", (unsigned int) home_configuration.size());
			home_configuration_ = std::vector<double>(7,0.0);
		}
		init( move_home_topic);
	}

private:
	void init( const std::string &move_home_topic)
	{
		// register callbacks
		move_home_action_server_.registerGoalCallback(boost::bind(&move_home_server::move_home_goal_callback, this));
		move_home_action_server_.registerPreemptCallback(boost::bind(&move_home_server::move_home_preempt_callback, this));

		// start action server
		ROS_INFO("Start server");
		move_home_action_server_.start();

		// start service
		move_home_service_server_ = node_handle_.advertiseService( move_home_topic, &move_home_server::move_home_callback, this);
	}

public:
	/**
	 * destructor
	 */
	~move_home_server()
	{}

private:
	bool move_home_callback( ros_common_robin_msgs::executeRequest &request, ros_common_robin_msgs::executeResponse &result )
	{
		ROS_INFO("got move home service request");

		// plan path above pick pose
		moveit::planning_interface::MoveGroup::Plan plan_home;

		ROS_INFO("Plan path");
		unsigned int ii=0;
		while(ii<5){
			if( lwa_.plan_continuous_path_.plan_to_configuration( plan_home, home_configuration_)){
				ROS_INFO("Found valid path");
				break;
			}
			if( !ros::ok() )
				break;
			ii++;
		}
		if( !ros::ok())
			return true;

		if(ii==5)
		{
			result.error_message = "Can't find a valid path";
			ROS_ERROR("%s", result.error_message.c_str());
			return true;
		}

		// visualize plan
		lwa_.plan_continuous_path_.visualize_plan();

		// move home
		ROS_INFO("Execution");
		std::string error_message = lwa_.execute_continuous_path_.execute_path_blocking( plan_home);
		if( !error_message.empty() ){
			result.error_message = "Execution failed";
			ROS_ERROR("%s", result.error_message.c_str());
		}

		return true;
	}

	void move_home_goal_callback( void)
	{
		ROS_INFO("got move home action request");
		omnirob_robin_moveit::move_homeResult result;

		// accept new goal
		move_home_action_server_.acceptNewGoal();

		// plan path above pick pose
		moveit::planning_interface::MoveGroup::Plan plan_home;

		ROS_INFO("Plan path");
		unsigned int ii=0;
		while(ii<5){
			if( lwa_.plan_continuous_path_.plan_to_configuration( plan_home, home_configuration_)){
				ROS_INFO("Found valid path");
				break;
			}
			if( (!ros::ok()) || (!move_home_action_server_.isActive()))
				break;
			ii++;
		}
		if( !ros::ok())
			return;
		if( !move_home_action_server_.isActive())
		{
			result.error_message = "Action canceled";
			ROS_ERROR("%s", result.error_message.c_str());
			move_home_action_server_.setAborted( result);
			return;
		}
		if(ii==5)
		{
			result.error_message = "Can't find a valid path";
			ROS_ERROR("%s", result.error_message.c_str());
			move_home_action_server_.setAborted( result);
			return;
		}

		// visualize plan
		lwa_.plan_continuous_path_.visualize_plan();

		// move home
		ROS_INFO("Execution");
		std::string error_message = lwa_.execute_continuous_path_.execute_path_blocking( plan_home);
		if( !error_message.empty() ){
			result.error_message = "Execution of plan failed";
			ROS_ERROR("%s", result.error_message.c_str());

			// cancel request
			move_home_action_server_.setAborted( result);
			return;
		}

		// finish action
		move_home_action_server_.setSucceeded( result);
	}

	void move_home_preempt_callback( void)
	{
		ROS_ERROR("move home action is preempted!");
		move_home_action_server_.setPreempted();
	}

private:
	// node handle
	ros::NodeHandle node_handle_;

	// action servers
	actionlib::SimpleActionServer<omnirob_robin_moveit::move_homeAction> move_home_action_server_;

	// service server
	ros::ServiceServer move_home_service_server_;

	// lwa control
	lwa_planner_and_executer lwa_;
	std::vector<double> home_configuration_;
};

int main(int argc, char **argv)
{

	// initialize ros node
	ros::init(argc, argv, "move_home_action_server");
	
	// parse poses from srdf
	std::string robot_description;
	ros::param::get("/robot_description", robot_description);
	if( robot_description.empty())
	{
		ROS_ERROR("Can't find robot description file");
		return -1;
	}
	std::string robot_description_semantic;
	ros::param::get("/robot_description_semantic", robot_description_semantic);
	if( robot_description_semantic.empty())
	{
		ROS_ERROR("Can't find semantic robot description file");
		return -1;
	}

	rdf_loader::RDFLoader rdf_loader( robot_description, robot_description_semantic);
	std::vector<srdf::Model::GroupState> group_states = rdf_loader.getSRDF()->getGroupStates();

	// produce one move to server for each defined state
	unsigned int cnt_lwa_states=0;

	std::vector<move_home_server*> move_home_servers;
	std::vector<double> configuration(7,0.0);
	for( unsigned int index=0; index<group_states.size(); index++)
	{
		if( group_states[index].group_.compare("lwa")==0 )
		{
			configuration[0] = group_states[index].joint_values_["lwa/joint_1"][0];
			configuration[1] = group_states[index].joint_values_["lwa/joint_2"][0];
			configuration[2] = group_states[index].joint_values_["lwa/joint_3"][0];
			configuration[3] = group_states[index].joint_values_["lwa/joint_4"][0];
			configuration[4] = group_states[index].joint_values_["lwa/joint_5"][0];
			configuration[5] = group_states[index].joint_values_["lwa/joint_6"][0];
			configuration[6] = group_states[index].joint_values_["lwa/joint_7"][0];

			cnt_lwa_states++;
			ROS_INFO("%u found state: %s = (%f,%f,%f,%f,%f,%f,%f) rad", cnt_lwa_states, group_states[index].name_.c_str(),
					 configuration[0], configuration[1], configuration[2], configuration[3], configuration[4], configuration[5], configuration[6] );
			move_home_server* tmp_move_home_server = new move_home_server( configuration, "/move_group/move_lwa_to/" + group_states[index].name_);
			move_home_servers.push_back(tmp_move_home_server);
		}
	}

	// required for communicating with moveit
	ros::AsyncSpinner spinner(2*cnt_lwa_states);
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
	std::string table_pose_frame = "/base_link";

	lwa_continuous_path_planner planner;
	ROS_INFO("add collision object");
	planner.add_static_object( table_id, table_primitive, table_pose, table_pose_frame);

	ros::spin();

	// free all servers
	for( unsigned int index=0; index<move_home_servers.size(); index++)
	{
		delete move_home_servers[index];
	}

	return 0;

}// main
