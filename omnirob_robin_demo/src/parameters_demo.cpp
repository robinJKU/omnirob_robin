#include <ros/ros.h>
#include <ros_common_robin_tools/common_tools.h>
#include <sstream> 
#include <iostream>
using namespace std;

//services und messages

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include <ros_common_robin_msgs/localization.h>
#include <tf/transform_listener.h>

double MAX_LIN_VEL = 0.25;
double MAX_ANG_VEL = 30;

ros::Publisher cmd_vel_publisher;
ros::ServiceServer base_demo_server_x, base_demo_server_y, base_demo_server_theta;
double x_distance,y_distance,ang_distance,max_lin_speed,max_ang_speed,max_lin_acc,max_ang_acc,STEP_TIME;

tf::TransformListener* pListener;
tf::Transform odom_to_base_i_,odom_to_base_f_;
tf::StampedTransform odom_to_base_i, odom_to_base_f;
tf::Transform basei_to_odom_, basei_to_basef_;


double pos_x, pos_y, yaw;


void move_base(double vx, double vy, double omega){
    geometry_msgs::Twist msg;
    if (sqrt(vx*vx + vy*vy) > MAX_LIN_VEL || fabs(omega) > MAX_ANG_VEL){
        ROS_INFO("speed limit reached");
    } else {
        msg.linear.x = vx;
        msg.linear.y = vy;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = omega*3.141592654/180;
        cmd_vel_publisher.publish(msg);    
    }    
}

//move base starting vom inital coordinate frame relative
void move_base_rel_pos_x(double X){

    double distance = abs(X);
    double distance_half=distance/2;
    
    double t_trans=max_lin_speed/max_lin_acc;
    double distance_trans=0.5*max_lin_acc*t_trans*t_trans;
    double step_time=STEP_TIME;
    double Vx;
    double t=0;

    if (distance_half<=distance_trans){

	    double time=sqrt(2*distance_half/max_lin_acc);

	    while (t < time) {
	    
		Vx= max_lin_acc*t;
                cout<<"time t= "<<t;
                cout<<" ;Vx= "<<Vx<<endl;
		if (X>0){
		move_base(Vx, 0, 0);
		}
		else{
		move_base(-Vx, 0, 0);
                }
		t += step_time;
		ros::Duration(step_time).sleep();
	    }
	    double V_top=Vx;
	    while (t<=2*time){

		Vx= V_top-max_lin_acc*(t-time);
                cout<<"time t= "<<t;
                cout<<" ;Vx= "<<Vx<<endl;
		if (X>0){
		move_base(Vx, 0, 0);
		}
		else{
		move_base(-Vx, 0, 0);
                }
		t += step_time;
		ros::Duration(step_time).sleep();
	    }

	    move_base(0, 0, 0);

	    }
    else{

	    while (t < t_trans) {
	    
		Vx= max_lin_acc*t;
                cout<<"time t= "<<t;
                cout<<" ;Vx= "<<Vx<<endl;
		if (X>0){
		move_base(Vx, 0, 0);
		}
		else{
		move_base(-Vx, 0, 0);
                }
		t += step_time;
		ros::Duration(step_time).sleep();
	    }

	    double distance_diff=distance_half-distance_trans;
	    double time_cost=distance_diff/max_lin_speed;

	    while (t<(t_trans+2*time_cost)){

		Vx= max_lin_speed;
                cout<<"time t= "<<t;
                cout<<" ;Vx= "<<Vx<<endl;
		if (X>0){
		move_base(Vx, 0, 0);
		}
		else{
		move_base(-Vx, 0, 0);
                }
		t += step_time;
		ros::Duration(step_time).sleep();
	    }

	    double t_tot=2*t_trans+2*time_cost;

	    while (t<=t_tot){

		Vx= max_lin_speed-max_lin_acc*(t-t_trans-2*time_cost);
                cout<<"time t= "<<t;
                cout<<" ;Vx= "<<Vx<<endl;
		if (X>0){
		move_base(Vx, 0, 0);
		}
		else{
		move_base(-Vx, 0, 0);
                }
		t += step_time;
		ros::Duration(step_time).sleep();
	    }
	    move_base(0, 0, 0);

    }
}


void move_base_rel_pos_y(double Y){

    double distance = abs(Y);
    double distance_half=distance/2;
    
    double t_trans=max_lin_speed/max_lin_acc;
    double distance_trans=0.5*max_lin_acc*t_trans*t_trans;
    double step_time=STEP_TIME;
    double Vy;
    double t=0;

    if (distance_half<=distance_trans){

	    double time=sqrt(2*distance_half/max_lin_acc);

	    while (t < time) {
	    
		Vy= max_lin_acc*t;
                cout<<"time t= "<<t;
                cout<<" ;Vy= "<<Vy<<endl;
		if (Y>0){
		move_base(0, Vy, 0);
		}
		else{
		move_base(0, -Vy, 0);
                }
		t += step_time;
		ros::Duration(step_time).sleep();
	    }
	    double V_top=Vy;
	    while (t<=2*time){

		Vy= V_top-max_lin_acc*(t-time);
                cout<<"time t= "<<t;
                cout<<" ;Vy= "<<Vy<<endl;
		if (Y>0){
		move_base(0, Vy, 0);
		}
		else{
		move_base(0, -Vy, 0);
                }
		t += step_time;
		ros::Duration(step_time).sleep();
	    }

	    move_base(0, 0, 0);

	    }
    else{

	    while (t < t_trans) {
	    
		Vy= max_lin_acc*t;
                cout<<"time t= "<<t;
                cout<<" ;Vy= "<<Vy<<endl;
		if (Y>0){
		move_base(0, Vy, 0);
		}
		else{
		move_base(0, -Vy, 0);
                }
		t += step_time;
		ros::Duration(step_time).sleep();
	    }

	    double distance_diff=distance_half-distance_trans;
	    double time_cost=distance_diff/max_lin_speed;

	    while (t<(t_trans+2*time_cost)){

		Vy= max_lin_speed;
                cout<<"time t= "<<t;
                cout<<" ;Vy= "<<Vy<<endl;
		if (Y>0){
		move_base(0, Vy, 0);
		}
		else{
		move_base(0, -Vy, 0);
                }
		t += step_time;
		ros::Duration(step_time).sleep();
	    }

	    double t_tot=2*t_trans+2*time_cost;

	    while (t<=t_tot){

		Vy= max_lin_speed-max_lin_acc*(t-t_trans-2*time_cost);
                cout<<"time t= "<<t;
                cout<<" ;Vy= "<<Vy<<endl;
		if (Y>0){
		move_base(0, Vy, 0);
		}
		else{
		move_base(0, -Vy, 0);
                }
		t += step_time;
		ros::Duration(step_time).sleep();
	    }
	    move_base(0, 0, 0);

    }
}


void move_base_rel_pos_theta(double THETA){

    double distance = abs(THETA);
    double distance_half=distance/2;
    
    double t_trans=max_ang_speed/max_ang_acc;
    double distance_trans=0.5*max_ang_acc*t_trans*t_trans;
    double step_time=STEP_TIME;
    double omega;
    double t=0;

    if (distance_half<=distance_trans){

	    double time=sqrt(2*distance_half/max_ang_acc);

	    while (t < time) {
	    
		omega= max_ang_acc*t;
                cout<<"time t= "<<t;
                cout<<" ;OMEGA= "<<omega<<endl;
		if (THETA>0){
		move_base(0, 0, omega);
		}
		else{
		move_base(0, 0, -omega);
                }
		t += step_time;
		ros::Duration(step_time).sleep();
	    }
	    double V_top=omega;
	    while (t<=2*time){

		omega= V_top-max_ang_acc*(t-time);
                cout<<"time t= "<<t;
                cout<<" ;OMEGA= "<<omega<<endl;
		if (THETA>0){
		move_base(0, 0, omega);
		}
		else{
		move_base(0, 0, -omega);
                }
		t += step_time;
		ros::Duration(step_time).sleep();
	    }

	    move_base(0, 0, 0);

	    }
    else{

	    while (t < t_trans) {
	    
		omega= max_ang_acc*t;
                cout<<"time t= "<<t;
                cout<<" ;OMEGA= "<<omega<<endl;
		if (THETA>0){
		move_base(0, 0, omega);
		}
		else{
		move_base(0, 0, -omega);
                }
		t += step_time;
		ros::Duration(step_time).sleep();
	    }

	    double distance_diff=distance_half-distance_trans;
	    double time_cost=distance_diff/max_ang_speed;

	    while (t<(t_trans+2*time_cost)){

		omega= max_ang_speed;
                cout<<"time t= "<<t;
                cout<<" ;OMEGA= "<<omega<<endl;
		if (THETA>0){
		move_base(0, 0, omega);
		}
		else{
		move_base(0, 0, -omega);
                }
		t += step_time;
		ros::Duration(step_time).sleep();
	    }

	    double t_tot=2*t_trans+2*time_cost;

	    while (t<=t_tot){

		omega= max_ang_speed-max_ang_acc*(t-t_trans-2*time_cost);
                cout<<"time t= "<<t;
                cout<<" ;OMEGA= "<<omega<<endl;
		if (THETA>0){
		move_base(0, 0, omega);
		}
		else{
		move_base(0, 0, -omega);
                }
		t += step_time;
		ros::Duration(step_time).sleep();
	    }
	    move_base(0, 0, 0);

    }
}


bool base_demo_callback_x(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){

	ROS_INFO("base_demo_srv x called");

	try
	{
	pListener->waitForTransform(
	"/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
	pListener->lookupTransform (
	"/odom", "/base_link", ros::Time(0), odom_to_base_i);
	}
	catch (tf::TransformException ex)
	{
	ROS_WARN("Could not get initial transform from odom to base, %s", ex.what());
	}
	odom_to_base_i_ = odom_to_base_i;

	//Move the base

	move_base_rel_pos_x(x_distance);

	try
	{
	pListener->waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
	pListener->lookupTransform ("/odom", "/base_link", ros::Time(0), odom_to_base_f);
	}
	catch (tf::TransformException ex)
	{
	ROS_WARN("Could not get final transform from odom to base, %s", ex.what());
	}
	odom_to_base_f_ = odom_to_base_f;

	//Final Position

        basei_to_odom_=odom_to_base_i_.inverse();
	basei_to_basef_=basei_to_odom_*odom_to_base_f_;

        pos_x = basei_to_basef_.getOrigin().getX();
	pos_y = basei_to_basef_.getOrigin().getY();
  	yaw = tf::getYaw(basei_to_basef_.getRotation());

	cout<<"Position x= "<<pos_x<<endl;
	cout<<"Position y= "<<pos_y<<endl;
	cout<<"Angle yaw= "<<yaw<<endl;

	return true;
}

bool base_demo_callback_y(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){

	ROS_INFO("base_demo_srv y called");

	try
	{
	pListener->waitForTransform(
	"/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
	pListener->lookupTransform (
	"/odom", "/base_link", ros::Time(0), odom_to_base_i);
	}
	catch (tf::TransformException ex)
	{
	ROS_WARN("Could not get initial transform from odom to base, %s", ex.what());
	}
	odom_to_base_i_ = odom_to_base_i;

	//Move the base

	move_base_rel_pos_y(y_distance);

	try
	{
	pListener->waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
	pListener->lookupTransform ("/odom", "/base_link", ros::Time(0), odom_to_base_f);
	}
	catch (tf::TransformException ex)
	{
	ROS_WARN("Could not get final transform from odom to base, %s", ex.what());
	}
	odom_to_base_f_ = odom_to_base_f;

	//Final Position

        basei_to_odom_=odom_to_base_i_.inverse();
	basei_to_basef_=basei_to_odom_*odom_to_base_f_;

        pos_x = basei_to_basef_.getOrigin().getX();
	pos_y = basei_to_basef_.getOrigin().getY();
  	yaw = tf::getYaw(basei_to_basef_.getRotation());

	cout<<"Position x= "<<pos_x<<endl;
	cout<<"Position y= "<<pos_y<<endl;
	cout<<"Angle yaw= "<<yaw<<endl;

	return true;

}

bool base_demo_callback_theta(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){

	ROS_INFO("base_demo_srv theta called");

	try
	{
	pListener->waitForTransform(
	"/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
	pListener->lookupTransform (
	"/odom", "/base_link", ros::Time(0), odom_to_base_i);
	}
	catch (tf::TransformException ex)
	{
	ROS_WARN("Could not get initial transform from odom to base, %s", ex.what());
	}
	odom_to_base_i_ = odom_to_base_i;

	//Move the base

	move_base_rel_pos_theta(ang_distance);

	try
	{
	pListener->waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(3.0));
	pListener->lookupTransform ("/odom", "/base_link", ros::Time(0), odom_to_base_f);
	}
	catch (tf::TransformException ex)
	{
	ROS_WARN("Could not get final transform from odom to base, %s", ex.what());
	}
	odom_to_base_f_ = odom_to_base_f;

	//Final Position

        basei_to_odom_=odom_to_base_i_.inverse();
	basei_to_basef_=basei_to_odom_*odom_to_base_f_;

        pos_x = basei_to_basef_.getOrigin().getX();
	pos_y = basei_to_basef_.getOrigin().getY();
  	yaw = tf::getYaw(basei_to_basef_.getRotation());

	cout<<"Position x= "<<pos_x<<endl;
	cout<<"Position y= "<<pos_y<<endl;
	cout<<"Angle yaw= "<<yaw<<endl;

	return true;
}


int main( int argc, char** argv) {

	// initialize node

	ros::init(argc, argv, "positioning_demo");
	ros::NodeHandle node_handle;

	pListener = new(tf::TransformListener);

        ros::param::get("/x_distance", x_distance);
        ros::param::get("/y_distance", y_distance);
        ros::param::get("/ang_distance", ang_distance);
        ros::param::get("/max_lin_speed", max_lin_speed);
        ros::param::get("/max_ang_speed", max_ang_speed);
        ros::param::get("/max_lin_acc", max_lin_acc);
        ros::param::get("/max_ang_acc", max_ang_acc);
        ros::param::get("/STEP_TIME", STEP_TIME);

	//publisher
        cmd_vel_publisher = node_handle.advertise<geometry_msgs::Twist>("/omnirob_robin/base/drives/control/cmd_vel", 1);

	//Service server
	base_demo_server_x = node_handle.advertiseService("/omnirob_robin/base_demo_move_srv_x", base_demo_callback_x); 
	base_demo_server_y = node_handle.advertiseService("/omnirob_robin/base_demo_move_srv_y", base_demo_callback_y);
	base_demo_server_theta = node_handle.advertiseService("/omnirob_robin/base_demo_move_srv_theta", base_demo_callback_theta);

	ros::spin();
	
}
