#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64.h>

void spinThread( void ){
	ros::spin();
}

int main( int argc, char** argv) {

	// init ros node
	ros::init(argc, argv, "follow_joint_trajectory_controller_test_client");
	
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("follow_joint_trajectory_controller");
	boost::thread spin_thread(&spinThread);
	
	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();
	
	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory.joint_names.push_back("lwa/joint_1");
	goal.trajectory.joint_names.push_back("lwa/joint_2");
	goal.trajectory.joint_names.push_back("lwa/joint_3");
	goal.trajectory.joint_names.push_back("lwa/joint_4");
	goal.trajectory.joint_names.push_back("lwa/joint_5");
	goal.trajectory.joint_names.push_back("lwa/joint_6");
	goal.trajectory.joint_names.push_back("lwa/joint_7");
	
	std::vector<double> joint_vector(7);
	goal.trajectory.points.resize(5);
	
	joint_vector[0]=0.0; joint_vector[1]=0.0; joint_vector[2]=0.0; joint_vector[3]=0.0; joint_vector[4]=0.0; joint_vector[5]=0.0; joint_vector[6]=0.0; joint_vector[7]=0.0;
	goal.trajectory.points[0].positions = joint_vector;
	joint_vector[0]=0.3; joint_vector[1]=0.0; joint_vector[2]=0.4; joint_vector[3]=0.0; joint_vector[4]=0.7; joint_vector[5]=0.0; joint_vector[6]=0.9; joint_vector[7]=0.0;
	goal.trajectory.points[1].positions = joint_vector;
	joint_vector[0]=0.5; joint_vector[1]=0.0; joint_vector[2]=0.9; joint_vector[3]=0.0; joint_vector[4]=1.0; joint_vector[5]=0.0; joint_vector[6]=0.3; joint_vector[7]=0.0;
	goal.trajectory.points[2].positions = joint_vector;
	joint_vector[0]=-0.5; joint_vector[1]=0.0; joint_vector[2]=-0.9; joint_vector[3]=0.0; joint_vector[4]=-1.0; joint_vector[5]=0.0; joint_vector[6]=-0.3; joint_vector[7]=0.0;
	goal.trajectory.points[3].positions = joint_vector;
	joint_vector[0]=0.0; joint_vector[1]=0.0; joint_vector[2]=0.0; joint_vector[3]=0.0; joint_vector[4]=0.0; joint_vector[5]=0.0; joint_vector[6]=0.0; joint_vector[7]=0.0;
	goal.trajectory.points[4].positions = joint_vector;
	
	ac.sendGoal(goal);
	
	//wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(0.0));
    
    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
      ROS_INFO("Action did not finish before the time out.");
    
    // shutdown the node and join the thread back before exiting
    ros::shutdown();
    spin_thread.join();

  //exit
	
}// main
