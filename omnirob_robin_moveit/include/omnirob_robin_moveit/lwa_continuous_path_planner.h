#ifndef __LWA_CONTINOUS_PATH_PLANNER_H
#define __LWA_CONTINOUS_PATH_PLANNER_H

// ros
#include <ros/ros.h>

// moveit
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <ros_common_robin_tools/common_tools.h>
#include <ros_common_robin_tools/common_geometry_tools.h>
#include <omnirob_robin_moveit/moveit_tools.h>




/**
 * This class is a client for moveits move_group node. The methods are restricted to path planning.
 * It configures the node properly and simplifies the communication with the node. This includes not only the path planning but also the scene manipulation.
 */
class lwa_continuous_path_planner{
	public:
		/**
		 * constructor
		 */
		lwa_continuous_path_planner():
			node_handle_(),
			pose_transformer_(),
			visualize_plan_after_planning_(true),
			spinner_(1)
		{
			// construct moveit objects
			lwa_move_group_ = new moveit::planning_interface::MoveGroup("lwa");
			planning_scene_interface_ = new moveit::planning_interface::PlanningSceneInterface();

			bool is_configured;
			node_handle_.param<bool>("/move_group/is_configured", is_configured, false);
			if( !is_configured) // configure move group node
			{
				lwa_move_group_->setPlannerId( "ESTkConfigDefault");
				lwa_move_group_->allowReplanning( true);

				double planning_time = 20.0; // s
				lwa_move_group_->setPlanningTime( planning_time);

				unsigned int num_planning_attemtps=10;
				lwa_move_group_->setNumPlanningAttempts( num_planning_attemtps);

				// configure workspace -> chosen such, that the whole arena is included
				double workspace_min_x=-5.0, workspace_min_y=-5.0, workspace_min_z=-5.0; // m
				double workspace_max_x= 5.0, workspace_max_y= 5.0, workspace_max_z= 5.0; // m
				lwa_move_group_->setWorkspace( workspace_min_x, workspace_min_y, workspace_min_z,
											   workspace_max_x, workspace_max_y, workspace_max_z); // defined in planning frame
				// lwa_move_group_->getPlanningFrame(); // world - is automatically chosen by the first link specified in the urdf file

				node_handle_.setParam( "/move_group/is_configure", true);

				lwa_names_.push_back("lwa/link_1");
				lwa_names_.push_back("lwa/link_2");
				lwa_names_.push_back("lwa/link_3");
				lwa_names_.push_back("lwa/link_4");
				lwa_names_.push_back("lwa/link_5");
				lwa_names_.push_back("lwa/link_6");
				lwa_names_.push_back("lwa/link_7");

				// wait for connection
				ros::NodeHandle node_handle;
				ros::Publisher temp_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
				ROS_INFO("wait for at least one subscriber on topic /planning_scene"); // ros needs some time for establishing the connection
				common_tools::wait_until_publisher_is_connected( temp_publisher);// this is not the used publisher (@see member variables PlanningSceneInterface), but it is used as measurement who long it takes to establish the connection
			}

			// the state monitor is usually started by the getCurrentState function, however
			// this function is really slow (sleeps for some secounds). Starting the state monitor
			// improves the performance a lot
			lwa_move_group_->startStateMonitor();

			// initialize publisher
			visualize_trajectory_publisher_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
			visualize_robot_state_publisher_ = node_handle_.advertise<moveit_msgs::DisplayRobotState>("/robot_state", 1, true);

			// start spinning - required by the move group interface
			spinner_.start();

		}// constructor

		/**
		 * destructor
		 */
		~lwa_continuous_path_planner()
		{
			if( lwa_move_group_!=NULL)
				delete lwa_move_group_;

			if( planning_scene_interface_!=NULL)
				delete planning_scene_interface_;
		}// destructor


	public: // planning interface
		/**
		 * This blocking function plans a path to the specified lwa configuration.
		 * @param plan: Returns the motion plan
		 * @param configuration: Vector which specifies the desired configuration of the lwa with its 7 elements.
		 * @return true on success
		 * @see moveit::planning_interface::MoveGroup::plan
		 */
		bool plan_to_configuration( moveit::planning_interface::MoveGroup::Plan &plan, const std::vector<double> &configuration)
		{
			// check input
			if( configuration.size()!=7 ){
				ROS_ERROR("Unexpected number of target configurations, got %u expected %u. Cancel motion plan request", (unsigned int) configuration.size(), (unsigned int) 7);
				return false;
			}

			// set goal values
			if( !lwa_move_group_->setJointValueTarget( configuration) ){
				ROS_WARN("Goal values are out of bounds q=(%f,%f,%f,%f,%f,%f,%f) , try to solve plan.", configuration[0], configuration[1], configuration[2], configuration[3], configuration[4], configuration[5], configuration[6] );
			}

			bool success = lwa_move_group_->plan( plan);
			if( success){
				last_plan_=plan;
				if( visualize_plan_after_planning_)
					visualize_plan( plan);
			}
			return success;
		}
		/**
		 * This blocking function plans a path from a specified configuration to a specified configuration.
		 * @param plan: Returns the motion plan
		 * @param start_configuration: Vector which specifies the initial configuration of the lwa with its 7 elements.
		 * @param goal_configuration: Vector which specifies the desired configuration of the lwa with its 7 elements.
		 * @return true on success
		 * @see moveit::planning_interface::MoveGroup::plan
		 */
		bool plan_to_configuration( moveit::planning_interface::MoveGroup::Plan &plan, const std::vector<double> &start_configuration, const std::vector<double> &goal_configuration)
		{
			// check input
			if( start_configuration.size()!=7 ){
				ROS_ERROR("Unexpected number of start configurations, got %u expected %u. Cancel motion plan request", (unsigned int) start_configuration.size(), (unsigned int) 7);
				return false;
			}
			if( goal_configuration.size()!=7 ){
				ROS_ERROR("Unexpected number of goal configurations, got %u expected %u. Cancel motion plan request", (unsigned int) goal_configuration.size(), (unsigned int) 7);
				return false;
			}

			// set start values
			set_start_state( start_configuration);

			// plan path
			return plan_to_configuration( plan, goal_configuration);
		}

	private:
		bool plan_path( moveit::planning_interface::MoveGroup::Plan &plan, const geometry_msgs::Pose &goal_pose, const std::string &target_frame = "", bool oriented = false)
		{
			std::string t_frame=target_frame;

			// check inputs
			std::string planning_frame = lwa_move_group_->getPlanningFrame();

			if( t_frame.empty() )
				t_frame = planning_frame;

			if( !common_geometry_tools::pose_is_valid( goal_pose) ){
				ROS_ERROR("Specified goal pose is invalid, cancel motion plan request");
				return false;
			}

			// transform pose to planning frame
			geometry_msgs::Pose goal_pose_planning_frame;
			if( t_frame.compare( planning_frame)==0 ){
				goal_pose_planning_frame = goal_pose;
			}else{
				goal_pose_planning_frame = pose_transformer_.transform_pose( goal_pose, t_frame, planning_frame);
				if( !common_geometry_tools::pose_is_valid( goal_pose_planning_frame) )
					return false;
			}

			if(oriented){
				/*
				moveit_msgs::Constraints constraints;
				constraints.name = "gripper constrain";
				moveit_msgs::OrientationConstraint gripper_constrain;

				gripper_constrain.header.frame_id = "odom";
				gripper_constrain.link_name = "gripper/palm_link";
				gripper_constrain.orientation.x = goal_pose_planning_frame.orientation.x;
				gripper_constrain.orientation.y = goal_pose_planning_frame.orientation.y;
				gripper_constrain.orientation.z = goal_pose_planning_frame.orientation.z;
				gripper_constrain.orientation.w = goal_pose_planning_frame.orientation.w;
				gripper_constrain.absolute_x_axis_tolerance = 1.0;
				gripper_constrain.absolute_y_axis_tolerance = 1.0;
				gripper_constrain.absolute_z_axis_tolerance = 0.15;
				gripper_constrain.weight = 1.0;
				constraints.orientation_constraints.resize(1);
				constraints.orientation_constraints[0] = gripper_constrain;

				lwa_move_group_->setPathConstraints(constraints);
				*/
				//lwa_move_group_->setPoseTarget( goal_pose_planning_frame );
				std::vector<geometry_msgs::Pose> waypoints;
				waypoints.push_back(goal_pose_planning_frame);

				double success = lwa_move_group_->computeCartesianPath(waypoints, 0.1, 1.2, plan.trajectory_);
				if (success != -1.0){
					last_plan_= plan;
					if( visualize_plan_after_planning_){
						visualize_plan( plan );
					}
					return true;
				}
				return false;
			}

			// plan path
			lwa_move_group_->setPoseTarget( goal_pose_planning_frame );

			bool success = lwa_move_group_->plan(plan);
			if( success){
				last_plan_=plan;
				if( visualize_plan_after_planning_)
					visualize_plan( plan);
			}
			return success;
		}

	public:
		/**
		* This blocking function plans a path to the specified goal pose of the end effector link.
		* @param plan: returns the planned path
		* @param goal_pose: Goal pose of the end effector link describet in target_frame
		* @param target_frame: The frame in which the goal pose is specified. default: planning_frame (world)
		* @return true on success
		* @see moveit::planning_interface::MoveGroup::plan
		*/
		bool plan_path_to_pose( moveit::planning_interface::MoveGroup::Plan &plan, const geometry_msgs::Pose &goal_pose, const std::string &target_frame = "")
		{
			// set start state
			visualize_current_state();
			set_start_state();
			visualize_current_state();

			// plan path
			return plan_path( plan, goal_pose, target_frame);
		}


		/**
		* This blocking function plans a path to the specified goal pose of the end effector link.
		* @param plan: returns the planned path
		* @param goal_pose: Goal pose of the end effector link describet in target_frame
		* @param target_frame: The frame in which the goal pose is specified. default: planning_frame (world)
		* @return true on success
		* @see moveit::planning_interface::MoveGroup::plan
		*/
		bool plan_path_to_poses( moveit::planning_interface::MoveGroup::Plan &plan, const std::vector<geometry_msgs::Pose> &goal_poses, const std::string &target_frame = "")
		{
			std::string t_frame=target_frame;

			// check inputs
			std::string planning_frame = lwa_move_group_->getPlanningFrame();
			if( t_frame.empty() )
				t_frame = planning_frame;
			for( unsigned int goal_ii=0; goal_ii<goal_poses.size(); goal_ii++)
			{
				if( !common_geometry_tools::pose_is_valid( goal_poses[goal_ii]) ){
					ROS_ERROR("Specified goal pose nr %u is invalid, cancel motion plan request", goal_ii);
					return false;
				}
			}

			// transform pose to planning frame
			std::vector<geometry_msgs::Pose> goal_poses_planning_frame;
			if( t_frame.compare( planning_frame)==0 )
			{
				goal_poses_planning_frame = goal_poses;
			}
			else
			{
				for( unsigned int goal_ii=0; goal_ii<goal_poses.size(); goal_ii++)
				{
					goal_poses_planning_frame.push_back( pose_transformer_.transform_pose( goal_poses[goal_ii], t_frame, planning_frame));
					if( !common_geometry_tools::pose_is_valid( goal_poses_planning_frame.back()) )
						return false;
				}
			}

			for( unsigned int goal_ii=0; goal_ii<goal_poses_planning_frame.size(); goal_ii++)
			{
				ROS_INFO("goal %u: pos = [%f,%f,%f], or = [%f,%f,%f,%f]",goal_ii,
					goal_poses_planning_frame[goal_ii].position.x,
					goal_poses_planning_frame[goal_ii].position.y,
					goal_poses_planning_frame[goal_ii].position.z,
					goal_poses_planning_frame[goal_ii].orientation.x,
					goal_poses_planning_frame[goal_ii].orientation.y,
					goal_poses_planning_frame[goal_ii].orientation.z,
					goal_poses_planning_frame[goal_ii].orientation.w );
			}

			// plan path
			lwa_move_group_->setPoseTargets( goal_poses_planning_frame);

			bool success = lwa_move_group_->plan(plan);
			if( success){
				last_plan_=plan;
				if( visualize_plan_after_planning_)
					visualize_plan( plan);
			}
			return success;
		}

		/**
 		 * This blocking function plans a path to the specified goal pose of the end effector link.
 		 * @param plan: returns the planned path
 		 * @param start_configuration: Vector which specifies the initial configuration of the lwa with its 7 elements.
 		 * @param goal_pose: Goal pose of the end effector link describet in target_frame
 		 * @param target_frame: The frame in which the goal pose is specified. default: planning_frame (world)
 		 * @return true on success
 		 * @see moveit::planning_interface::MoveGroup::plan
 		 */
 		bool plan_path_to_pose( moveit::planning_interface::MoveGroup::Plan &plan, std::vector<double> start_configuration, const geometry_msgs::Pose &goal_pose, const std::string &target_frame = "")
 		{
 			std::string t_frame=target_frame;

 			// check inputs
 			std::string planning_frame = lwa_move_group_->getPlanningFrame();
 			if( t_frame.empty() )
 				t_frame = planning_frame;
 			if( !common_geometry_tools::pose_is_valid( goal_pose) ){
 				ROS_ERROR("Specified goal pose is invalid, cancel motion plan request");
 				return false;
 			}
 			if( start_configuration.size()!=7){
 				ROS_ERROR("Unexpected number of goal configurations, got %u expected %u. Cancel motion plan request", (unsigned int) start_configuration.size(), (unsigned int) 7);
 			}

 			// set start configuration
 			visualize_current_state();
 			set_start_state( start_configuration);
 			visualize_current_state();

 			// plan path
 			return plan_path( plan, goal_pose, t_frame);
 		}

 		bool plan_path_to_pose_oriented( moveit::planning_interface::MoveGroup::Plan &plan, std::vector<double> start_configuration, const geometry_msgs::Pose &goal_pose, const std::string &target_frame = "")
			{
				std::string t_frame=target_frame;

				// check inputs
				std::string planning_frame = lwa_move_group_->getPlanningFrame();
				if( t_frame.empty() )
					t_frame = planning_frame;
				if( !common_geometry_tools::pose_is_valid( goal_pose) ){
					ROS_ERROR("Specified goal pose is invalid, cancel motion plan request");
					return false;
				}
				if( start_configuration.size()!=7){
					ROS_ERROR("Unexpected number of goal configurations, got %u expected %u. Cancel motion plan request", (unsigned int) start_configuration.size(), (unsigned int) 7);
				}

				moveit::core::RobotState start_state( *(lwa_move_group_->getCurrentState()));
				start_state.setJointGroupPositions( "lwa", start_configuration);
				moveit::core::robotStateToRobotStateMsg(start_state, plan.start_state_);
				plan.planning_time_ = 0.1;

				// set start configuration
				visualize_current_state();
				set_start_state( start_configuration);
				visualize_current_state();

				// plan path
				return plan_path( plan, goal_pose, t_frame, true);
			}

 		/**
		 * This blocking function plans a path to a set of goal poses of the end effector link.
		 * @param plan: returns the planned path
		 * @param start_configuration: Vector which specifies the initial configuration of the lwa with its 7 elements.
		 * @param goal_poses: Goal poses of the end effector link describet in target_frame
		 * @param target_frame: The frame in which the goal pose is specified. default: planning_frame (world)
		 * @return true on success
		 * @see moveit::planning_interface::MoveGroup::plan
		 */
		bool plan_path_to_poses( moveit::planning_interface::MoveGroup::Plan &plan, std::vector<double> start_configuration, const std::vector<geometry_msgs::Pose> &goal_poses, const std::string &target_frame = "")
		{
			std::string t_frame=target_frame;

			// check inputs
			std::string planning_frame = lwa_move_group_->getPlanningFrame();
			if( t_frame.empty() )
				t_frame = planning_frame;
			for( unsigned int goal_ii=0; goal_ii<goal_poses.size(); goal_ii++)
			{
				if( !common_geometry_tools::pose_is_valid( goal_poses[goal_ii]) ){
					ROS_ERROR("Specified goal pose nr %u is invalid, cancel motion plan request", goal_ii);
					return false;
				}
			}
			if( start_configuration.size()!=7){
				ROS_ERROR("Unexpected number of goal configurations, got %u expected %u. Cancel motion plan request", (unsigned int) start_configuration.size(), (unsigned int) 7);
			}

			// set start configuration
			visualize_current_state();
			set_start_state( start_configuration);
			visualize_current_state();

			// plan path
			return plan_path_to_poses(plan, goal_poses, t_frame);
		}

	public:
 		/**
 		 * Displays the current state on robot_state topic
 		 */
 		void visualize_current_state( void)
 		{
 			visualize_robot_state( *(lwa_move_group_->getCurrentState()));
 		}

	private:
 		void visualize_robot_state( const robot_state::RobotState &state)
		{
			moveit_msgs::DisplayRobotState cur_state_msg;
			robot_state::robotStateToRobotStateMsg( state, cur_state_msg.state);
			visualize_robot_state_publisher_.publish( cur_state_msg);
		}

	private: // state manipulation interface

 		void set_start_state( void)
 		{
 			lwa_move_group_->setStartStateToCurrentState();
 		}

		void set_start_state( const std::vector<double> &start_configuration){
			// hint: the get current state function listens on the joint_state topic and does not
			//       get the state from move_group
			moveit::core::RobotState start_state( *(lwa_move_group_->getCurrentState()));
			start_state.setJointGroupPositions( "lwa", start_configuration);
			lwa_move_group_->setStartState(start_state);

			visualize_robot_state( start_state);
		}


	public: // scene manipulation interface
		/**
		 * Returns the pose of a specific collision object.
		 */
		geometry_msgs::Pose get_object_pose( std::string object_id){
			std::vector<std::string> object_ids;
			object_ids.push_back( object_id);
			std::map<std::string,geometry_msgs::Pose> poses = planning_scene_interface_->getObjectPoses(object_ids);
			if( poses.size()!= 1)
				return geometry_msgs::Pose();

			return poses[object_id];
		}
		/**
		 * Returns a list of all known collision objects
		 */
		std::vector<std::string> object_list(){
			return planning_scene_interface_->getKnownObjectNames( false);
		}
		/**
		 * Check if a specific collision object is in the scene
		 */
		bool object_exists( std::string object_id){
			std::vector<std::string> object_ids = object_list();
			for( unsigned int object_ii=0; object_ii<object_ids.size(); object_ii++){
				if( object_ids[object_ii].compare( object_id)==0 )
					return true;
			}
			return false;
		}
		/**
		 * Adds a collision object to the planning scene.
		 */
		void add_static_object( std::string object_id, shape_msgs::SolidPrimitive primitive, geometry_msgs::Pose primitive_pose, std::string frame_id, bool print_known_objects=true ){
			moveit_msgs::CollisionObject collision_object;

			collision_object.header.stamp = ros::Time::now();
			collision_object.header.frame_id = frame_id;
			collision_object.id = object_id;
			collision_object.primitives.push_back( primitive);
			collision_object.primitive_poses.push_back( primitive_pose);
			collision_object.operation = collision_object.ADD;

			std::vector<moveit_msgs::CollisionObject> collision_objects;
			collision_objects.push_back(collision_object);

			planning_scene_interface_->addCollisionObjects( collision_objects);

			if( print_known_objects){
				ros::spinOnce();
				std::vector<std::string> known_objects = planning_scene_interface_->getKnownObjectNames( false);
				ROS_INFO("known_objects:");
				for(unsigned int ii=0; ii<known_objects.size(); ii++)
				{
					ROS_INFO("%u: %s", ii, known_objects[ii].c_str());
				}
			}

		}
		/**
		 * @param object_id: Id of the object which should be attached
		 * @param connecting_link: Id of the link on which the object should be attached. default: end effector link
		 */
		void attach_existing_object(const std::string &object_id, const std::string &connection_link=""){
			lwa_move_group_->attachObject( object_id, connection_link);
		}
		/**
		 * @param object_id: Id of the object which should be attached
		 * @param connecting_link: Id of the link on which the object should be attached
		 * @param touch_link: The set of links the object is allowed to touch without considering that a collision is specified by touch_links.
		 */
		void attach_existing_object(const std::string &object_id, const std::string &connection_link, const std::vector< std::string > &touch_links){
			lwa_move_group_->attachObject( object_id, connection_link, touch_links);
		}
		/**
		 * Add the specified object to the scene and attach it to the specified link.
		 * @see add_static_object
		 * @see attach_existing_object
		 */
		void add_attached_object(const std::string &object_id,
								 const shape_msgs::SolidPrimitive &primitive,
								 const geometry_msgs::Pose &primitive_pose,
								 const std::string &frame_id="",
								 const std::string &connection_link="" ){
			add_static_object( object_id, primitive, primitive_pose, frame_id);
			attach_existing_object( object_id, connection_link);
		}
		/**
		 * Add the specified object to the scene and attach it to the specified link.
		 * @see add_static_object
		 * @see attach_existing_object
		 */
		void add_attached_object(const std::string &object_id,
								 const shape_msgs::SolidPrimitive &primitive,
								 const geometry_msgs::Pose &primitive_pose,
								 const std::string &frame_id,
								 const std::string &connection_link,
								 const std::vector<std::string> &touch_links){
			add_static_object( object_id, primitive, primitive_pose, frame_id);
			attach_existing_object( object_id, connection_link, touch_links);
		}
		/**
		 * Detach an object from the robot
		 */
		void detach_object( const std::string &object_id){
			lwa_move_group_->detachObject( object_id);
		}
		/**
		 * Remove an collision object from the planning scene
		 */
		void remove_object( std::string object_id){
			std::vector<std::string> object_ids;
			object_ids.push_back( object_id);
			planning_scene_interface_->removeCollisionObjects( object_ids);
		}


	public: // path evaluation
		/**
		 * @param flag: Enable (true) or disable (false) visualization after planning.
		 */
		void visualize_plan_after_planning( bool flag ){
			visualize_plan_after_planning_ = flag;
		}
		/**
		 * This function visualize the last computed plan in rviz.
		 */
		void visualize_plan(){
			visualize_plan( last_plan_);
		}
		void visualize_last_plan(){
			visualize_plan( last_plan_);
		}
		/**
		 * This function visualize a given plan in rviz.
		 * @param plan: Default value is the last computed plan.
		 */
		void visualize_plan( moveit::planning_interface::MoveGroup::Plan plan ){
			moveit_msgs::DisplayTrajectory display_trajectory_msg;
			display_trajectory_msg.trajectory_start = plan.start_state_;
			display_trajectory_msg.trajectory.push_back(plan.trajectory_);

			visualize_trajectory_publisher_.publish( display_trajectory_msg);
		}// visualize plan

	public: // member variables
		// moveit
		moveit::planning_interface::MoveGroup *lwa_move_group_; // main communication layer for motion planning
		moveit::planning_interface::PlanningSceneInterface *planning_scene_interface_; // main communication layer for interacting with the scene

	private: // member variables
		// ros
		ros::NodeHandle node_handle_;
		ros::AsyncSpinner spinner_;

		// publisher
		ros::Publisher visualize_trajectory_publisher_;
		ros::Publisher visualize_robot_state_publisher_;

		// geometry
		common_geometry_tools::pose_transformer pose_transformer_;

		// state monitoring
		bool visualize_plan_after_planning_;
		moveit::planning_interface::MoveGroup::Plan last_plan_;
		std::vector<std::string> lwa_names_;
};

#endif
