#include <ros/ros.h>
#include <omnirob_robin_tools_ros/ros_tools.h>
#include "tf/transform_listener.h"


//services und messages

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "std_srvs/Empty.h"
#include <omnirob_robin_msgs/localization.h>
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

double MAX_LIN_VEL = 0.25;
double MAX_ANG_VEL = 0.4;
double STEP_TIME = 0.01;


ros::Publisher cmd_vel_publisher;

ros::ServiceClient marker_localization_client;

ros::ServiceServer base_demo_server;
ros::ServiceServer localize_server;

tf::Transform static_map_to_map;

tf::TransformListener * tf_listener;

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
        msg.angular.z = omega;
        cmd_vel_publisher.publish(msg);    
    }    
}

//move base starting vom inital coordinate frame relative
void move_base_rel_pos(double X, double Y, double Phi, bool stop){
    double distance = sqrt(X*X + Y*Y);
    double T = distance / MAX_LIN_VEL; //Zeit berechnen für Bahn
    Phi = Phi / 180.0 * M_PI;
    double T2 = fabs(Phi) / MAX_ANG_VEL;
    if (T2 > T){
        T = T2;
    }
    //parametrisieren der Bahn
    double Vx = X / T;
    double Vy = Y / T;
    double Omega = Phi / T;
    double phi = 0;
    double step_time = STEP_TIME;
    double t = STEP_TIME;
    if ( T < STEP_TIME) {
        t = T;
        step_time = T;
    }        
    while (t <= T) {
        double vx = Vx*cos(phi) + Vy*sin(phi);
        double vy = Vy*cos(phi) - Vx*sin(phi);
        move_base(vx, vy, Omega);
        t += step_time;
        phi = Omega*t;
        ros::Duration(step_time).sleep();
    }
    if(t-T > 0){
        ros::Duration(t-T).sleep();
    }
    if(stop){
        move_base(0, 0, 0);
    }
    
}

void move_base_half_circle(){
    double radius = 0.45;
    for (int i = 2; i < 180; i+=2){
        double x = radius - radius*cos(2.0/180.0*M_PI);
        double y = radius * sin(2.0/180.0*M_PI);
        move_base_rel_pos(-x, -y, 2, false);       
    } 
    move_base(0,0,0);  
}


bool base_demo_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
	ROS_INFO("base_demo_srv called");
    move_base_rel_pos(0.7, 0, 0, true);
    move_base_rel_pos(0.0, 0.7, 0, true);
    move_base_rel_pos(-0.7, 0, -90, true);
    move_base_half_circle();
    move_base_rel_pos(0.7, 0, -90, true);
    move_base_rel_pos(0.5, 0, 90, true);
    move_base_rel_pos(0, 0.5, -90, true);
	return true;
}

bool localize_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){ 
	// perform a global localization
    ROS_INFO("Perform global localization");
    omnirob_robin_msgs::localization marker_localization_msg;
    tf::StampedTransform transform;
    tf::Transform static_map_to_base_link;
    
    int i = 0;
    while(i < 6){
        marker_localization_client.call( marker_localization_msg);
        if( !marker_localization_msg.response.error_message.empty() ){
            ROS_ERROR("Can't initialize robot pose -reason: %s", marker_localization_msg.response.error_message.c_str());
            move_base_rel_pos(0, 0, 45, true);
            i++;
            continue;        
        }
        tf::poseMsgToTF(marker_localization_msg.response.base_link, static_map_to_base_link);
        ROS_INFO("vector = %f %f %f", static_map_to_base_link.getOrigin().getX(), static_map_to_base_link.getOrigin().getY(), static_map_to_base_link.getOrigin().getZ());
        break;        
    }
    
    try{
        tf_listener->lookupTransform("/base_link", "/map", ros::Time(0), transform);
        }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }    
    
    tf::poseMsgToTF(marker_localization_msg.response.base_link, static_map_to_base_link);
    static_map_to_map.mult(static_map_to_base_link, transform);
    
    tf::Quaternion quat = static_map_to_map.getRotation();
    
    //move_base_rel_pos(0, 0, -120, true);
    
    // "Finished scan, no marker detected"
    
	return true;
}


int main( int argc, char** argv) {

	// initialize node
	ros::init(argc, argv, "pick_and_place_demo");
	ros::NodeHandle node_handle;
	tf::TransformBroadcaster br;
    tf_listener = new tf::TransformListener();
    static_map_to_map.setOrigin(tf::Vector3(0,0,0));
    static_map_to_map.setRotation(tf::Quaternion(0,0,0,1));

	//publisher
	cmd_vel_publisher = node_handle.advertise<geometry_msgs::Twist>("/omnirob_robin/base/drives/control/cmd_vel", 10);
	//subscriber

	//Service server
	base_demo_server = node_handle.advertiseService("/omnirob_robin/base_demo_move_srv", base_demo_callback);
    localize_server = node_handle.advertiseService("/omnirob_robin/base_demo_localize_srv", localize_callback);
    
    //service clients    
    
    std::string marker_localization_topic = "/marker_localization";
    if( !omnirob_ros_tools::wait_for_service( marker_localization_topic, 10) ){
        return -1;
    }
    marker_localization_client = node_handle.serviceClient<omnirob_robin_msgs::localization>( marker_localization_topic);
  
    
    while(ros::ok){
        ros::Rate(100).sleep();
        br.sendTransform(tf::StampedTransform(static_map_to_map, ros::Time::now(), "static_map", "map"));
        ros::spinOnce();
    }
	
	ros::spin();
}

