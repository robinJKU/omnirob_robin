#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
class TurtlebotTeleop
{
public:
	TurtlebotTeleop();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void publish();
	ros::NodeHandle ph_, nh_;
	int linearX_, linearY_, angular_, deadman_axis_;
	double l_scaleX_, l_scaleY_, a_scale_;
	ros::Publisher vel_pub_;
	ros::Subscriber joy_sub_;
	geometry_msgs::Twist last_published_;
	boost::mutex publish_mutex_;
	bool deadman_pressed_;
	bool zero_twist_published_;
	ros::Timer timer_;
	};
		TurtlebotTeleop::TurtlebotTeleop():
		ph_("~"),
		linearX_(1),
		linearY_(0),
		angular_(2),
		deadman_axis_(4),
		l_scaleX_(0.2),
		l_scaleY_(0.2),
		a_scale_(0.8)
	{
	ph_.param("axis_linearX", linearX_, linearX_);
    ph_.param("axis_linearY", linearY_, linearY_);
	ph_.param("axis_angular", angular_, angular_);
	ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
	ph_.param("scale_angular", a_scale_, a_scale_);
	ph_.param("scale_linearX", l_scaleX_, l_scaleX_);
	ph_.param("scale_linearY", l_scaleY_, l_scaleY_);
	deadman_pressed_ = false;
	zero_twist_published_ = false;
	vel_pub_ = ph_.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TurtlebotTeleop::joyCallback, this);
	timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&TurtlebotTeleop::publish, this));
}

void TurtlebotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist vel;
	vel.angular.z = a_scale_*joy->axes[angular_];
	vel.linear.x = l_scaleX_*joy->axes[linearX_];
        vel.linear.y = l_scaleY_*joy->axes[linearY_];
	last_published_ = vel;
	deadman_pressed_ = joy->buttons[deadman_axis_];
}

void TurtlebotTeleop::publish()
{
	boost::mutex::scoped_lock lock(publish_mutex_);
	if (deadman_pressed_)
	{
		vel_pub_.publish(last_published_);
		zero_twist_published_=false;
	}
	else if(!deadman_pressed_ && !zero_twist_published_)
	{
		vel_pub_.publish(*new geometry_msgs::Twist());
		zero_twist_published_=true;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtlebot_teleop");
	TurtlebotTeleop turtlebot_teleop;
	ros::spin();
}
