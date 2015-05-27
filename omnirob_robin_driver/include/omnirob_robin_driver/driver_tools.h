#ifndef __OMNIROB_DRIVER_TOOLS_H
#define __OMNIROB_DRIVER_TOOLS_H

#include <ros/ros.h>
#include <omnirob_robin_tools_ros/ros_tools.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>

std_msgs::Float64MultiArray float_vector_to_multiarray( std::vector<float> vec ){
	std_msgs::Float64MultiArray multi_arrray;
	multi_arrray.data.resize( vec.size());
	for( unsigned int ii=0; ii<vec.size(); ii++){
		multi_arrray.data[ii] = vec[ii];
	}
	return multi_arrray;
}// float_vector_to_multiarray


class Pan_Tilt{
	private:
		// handles
		ros::NodeHandle node_handle;
		ros::Publisher cmd_joint_state_publisher;
		ros::Subscriber joint_state_subscriber;
		ros::ServiceClient start_motion_client;
		
		// state variables
		std::vector<float> joint_state;
		std::vector<float> last_commanded_joint_state;
		ros::Time joint_state_time_stamp;
		
		// static variables
		const static float POSITION_REACHED_EPSILON = 0.02; /** The position reached epsilon value is considered element wise and in rad */
		
	public:
		/**
		 * constructor
		 */
		Pan_Tilt(): node_handle(), joint_state(2), last_commanded_joint_state(2)
		{
			// initialize services
			std::string pan_tilt_start_motion_srv = "/omnirob_robin/pan_tilt/control/start_motion";
			if( !wait_for_service( pan_tilt_start_motion_srv) ){
				throw "start motion service does not exist";
			}else{
				start_motion_client = node_handle.serviceClient<std_srvs::Empty>( pan_tilt_start_motion_srv);
			}
			
			// initialize msg	
			cmd_joint_state_publisher = node_handle.advertise<std_msgs::Float64MultiArray>("/omnirob_robin/pan_tilt/control/commanded_joint_state", 5);
			joint_state_subscriber = node_handle.subscribe("/omnirob_robin/pan_tilt/state/joint_state_array", 10, &Pan_Tilt::joint_state_callback, this);
			
		}
		/**
		 * destructor
		 */
		~Pan_Tilt()
		{
			
		}
	
	public:
		/**
		 * Callback for the joint state subscriber
		 */
		void joint_state_callback( std_msgs::Float64MultiArray joint_state_msg ){
			if( joint_state_msg.data.size()==2 ){
				joint_state[0] = joint_state_msg.data[0];
				joint_state[1] = joint_state_msg.data[1];
				joint_state_time_stamp=ros::Time::now();
				
			}else{
				ROS_WARN("joint state callback - Msg data not proper sized: got %i, expected 2",  (int) joint_state_msg.data.size() );
			}
			
		}// joint state callback
		
		
	public:
		/**
		 * Blocks until timeout is reached or at least one node subscribes to the commanded joint state topic.
		 */
		bool wait_until_joint_state_publisher_is_connected( float timeout_in_seconds=10.0){
			if( !wait_until_publisher_is_connected( cmd_joint_state_publisher, timeout_in_seconds)){
				ROS_ERROR("Can't initialize pan commanded joint state publisher");
				return false;
			}
			return true;
		}// wait until joint state publisher is connected
	
	public:
		/**
		 * Returns if the considered position is in a rectangle arround goal position with edge length POSITION_REACHED_EPSILON
		 * @param goal_position: As default value, the last commanded joint is used
		 * @see POSITION_REACHED_EPSILON
		 */
		 bool is_position_reached( std::vector<float> goal_position = std::vector<float>()){
			 if( goal_position.size()==0 ){
				 goal_position = last_commanded_joint_state;
			 }
			 if( goal_position.size()!=2 ){
				 return false;
			 }
			 
			 bool position_is_reached=true;
			 for( unsigned int joint_ii=0; joint_ii<goal_position.size() && position_is_reached ; joint_ii++){
				position_is_reached = position_is_reached && ( fabs(joint_state[joint_ii]-goal_position[joint_ii])<POSITION_REACHED_EPSILON );
			 }
			 return position_is_reached;
			 
		 }// is position reached
	
	public:
		/**
		 * Sets goal position to target position and starts the motion
		 * @param goal_position: Vector with two elements. First Element corresponds to the pan angle, the second to the tilt angle.
		 */
		bool move_to_state( std::vector<float> goal_position ){
			if( goal_position.size()!=2 ){
				ROS_ERROR("Move To Pan Tilt Angle - unexpected size of input vector: got %i, expected 2", (int) goal_position.size());
				return false;
			}
			last_commanded_joint_state = goal_position;
			std_msgs::Float64MultiArray pan_goal_position_msgs = float_vector_to_multiarray( goal_position);
	
			cmd_joint_state_publisher.publish( pan_goal_position_msgs);
			ros::spinOnce();
			std_srvs::Empty empty_srvs;
			start_motion_client.call( empty_srvs);

			return true;
		}// move to state
		
		/**
		 * Sets goal position to target position, starts the motion and blocks until the motion is reached.
		 * @param goal_position: Vector with two elements. First Element corresponds to the pan angle, the second to the tilt angle.
		 * @param timeout_seconds: Maximal blocking time in seconds. A negative timeout means an infinity timeout.
		 */
		bool move_to_state_blocking( std::vector<float> goal_position, float timeout_seconds=-1.0 ){
			if( !move_to_state( goal_position)){
				return false;
			}
			
			ros::Rate sleep_rate(5.0);
			if( timeout_seconds>0.0 ){
				sleep_rate = ros::Rate( 10.0/timeout_seconds );
			}
			ros::spinOnce();
			
			unsigned int cnt=0;
			while( ros::ok() && (!is_position_reached()) ){
				sleep_rate.sleep();
				ros::spinOnce();
				
				if( timeout_seconds>0.0 ){
					cnt++;
					if( cnt>=10 ){
						break;
					}
				}
			}
			
			return is_position_reached();
		}// move to state blocking
		
		/**
		 * Returns the recent joint state
		 */
		std::vector<float> get_joint_state( void ){
			ros::spinOnce();
			return std::vector<float>( joint_state);
		}
		
};


#endif
