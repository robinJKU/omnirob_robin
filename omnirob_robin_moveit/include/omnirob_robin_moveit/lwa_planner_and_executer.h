#ifndef __LWA_MOTION_CONTROL_H
#define __LWA_MOTION_CONTROL_H

// ros
#include <ros/ros.h>

// omnirob robin
#include <omnirob_robin_moveit/lwa_continuous_path_planner.h>
#include <omnirob_robin_moveit/lwa_continuous_path_executer.h>
#include <omnirob_robin_moveit/lwa_point_to_point_path_executer.h>

class lwa_planner_and_executer{
	public:
		/**
		 * constructor
		 */
		lwa_planner_and_executer():
			plan_continuous_path_(),
			execute_continuous_path_(),
			execute_point_to_point_path_()
		{}
		/**
		 * destructor
		 */
		~lwa_planner_and_executer()
		{}

	public: // member variables:
		lwa_continuous_path_planner plan_continuous_path_;
		lwa_continuous_path_executer execute_continuous_path_;
		lwa_point_to_point_executer execute_point_to_point_path_;
};
#endif
