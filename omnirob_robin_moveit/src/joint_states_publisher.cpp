#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

  ros::Publisher *joint_states_pub;


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  //ROS_INFO("I heard: %s", msg->name[0].c_str()); 

  // %Tag(PUBLISH)%
  joint_states_pub->publish(msg);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_states_publisher");
  ros::NodeHandle nh;

  // subscribes the joint_states_topic from omnirob_robin gazebo
  ros::Subscriber sub = nh.subscribe("/omnirob_robin/joint_states", 1000, jointStatesCallback);


// publisher for our joint_state_topic
  ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);
  joint_states_pub = &pub;


  ros::spin();

  return 0;
}
