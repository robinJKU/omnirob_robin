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

	void cancel_goal(){
		pick_action_client_.cancelGoal();
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
	ros::init(argc, argv, "omnirob_moveit");

	pick_and_place_executer lwa_pick_and_place;

	std::string object_id = "cylinder_white";
	shape_msgs::SolidPrimitive object_primitive;
	object_primitive.type = object_primitive.CYLINDER;
	object_primitive.dimensions.push_back(2e-1); // height
	object_primitive.dimensions.push_back(4e-2); // radius

	ROS_INFO("give call");
	lwa_pick_and_place.pick_object( object_id, object_primitive, false);

	ros::spin();

	return 0;

}// main
