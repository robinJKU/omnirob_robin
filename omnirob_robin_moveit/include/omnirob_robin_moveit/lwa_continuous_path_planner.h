#ifndef __LWA_CONTINOUS_PATH_PLANNER_H
#define __LWA_CONTINOUS_PATH_PLANNER_H

// ros
#include <ros/ros.h>

// moveit
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>

#include <omnirob_robin_tools_ros/ros_tools.h>
#include <omnirob_robin_tools_ros/geometry_tools.h>
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
			visualize_plan_after_planning_(true)
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
				wait_until_publisher_is_connected( temp_publisher);// this is not the used publisher (@see member variables PlanningSceneInterface), but it is used as measurement who long it takes to establish the connection
			}

			// initialize publisher
			visualize_trajectory_publisher_ = node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

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
			std::string t_frame=target_frame;

			// check inputs
			std::string planning_frame = lwa_move_group_->getPlanningFrame();
			if( t_frame.empty() )
				t_frame = planning_frame;

			if( !omnirob_geometry_tools::pose_is_valid( goal_pose) ){
				ROS_ERROR("Specified goal pose is invalid, cancel motion plan request");
				return false;
			}

			// transform pose to planning frame
			geometry_msgs::Pose goal_pose_planning_frame;
			if( t_frame.compare( planning_frame)==0 ){
				goal_pose_planning_frame = goal_pose;
			}else{
				goal_pose_planning_frame = pose_transformer_.transform_pose( goal_pose, t_frame, planning_frame);
				if( !omnirob_geometry_tools::pose_is_valid( goal_pose_planning_frame) )
					return false;
			}

			// plan path
			lwa_move_group_->setPoseTarget( goal_pose_planning_frame);

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
 			if( !omnirob_geometry_tools::pose_is_valid( goal_pose) ){
 				ROS_ERROR("Specified goal pose is invalid, cancel motion plan request");
 				return false;
 			}
 			if( start_configuration.size()!=7){
 				ROS_ERROR("Unexpected number of goal configurations, got %u expected %u. Cancel motion plan request", (unsigned int) start_configuration.size(), (unsigned int) 7);
 			}

 			// set start configuration
 			set_start_state( start_configuration);

 			// plan path
 			return plan_path_to_pose(plan, goal_pose, t_frame);
 		}


	private: // state manipulation interface
		void set_start_state( const std::vector<double> &start_configuration){
			/*robot_state::RobotState start_state( *(lwa_move_group_->getCurrentState())); // copy current state
			std::map<std::string,double> start_configuration_map;
			for(unsigned int module=0; module<7; module++)
			{
				start_configuration_map[lwa_names_[module]]=start_configuration[module];
			}
			start_state.printStateInfo();
			start_state.set
			*/
			moveit::core::RobotState start_state( *(lwa_move_group_->getCurrentState()));
			start_state.setJointGroupPositions( "lwa", start_configuration);

			// start_state.setVariablePositions(start_configuration);
			// start_state.setVariablePositions( start_configuration_map);
			lwa_move_group_->setStartState(start_state);
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

		// publisher
		ros::Publisher visualize_trajectory_publisher_;

		// geometry
		omnirob_geometry_tools::pose_transformer pose_transformer_;

		// state monitoring
		bool visualize_plan_after_planning_;
		moveit::planning_interface::MoveGroup::Plan last_plan_;
		std::vector<std::string> lwa_names_;
};

#endif
