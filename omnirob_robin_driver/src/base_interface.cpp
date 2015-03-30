#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>


double x = 0.0;
double y = 0.0;
double yaw = 0.0;
double vx = 0.0;
double vy = 0.0;
double omegaz = 0.0;

ros::Time callback_time;
ros::Time start_time;
ros::Time stop_time;

// Initialize a callback to get velocity values:

void cmd_velCallback( const geometry_msgs::TwistConstPtr& cmd_vel) {

  vx = cmd_vel->linear.x;
  vy = cmd_vel->linear.y;
  omegaz = cmd_vel->angular.z;  
 
  callback_time = ros::Time::now();

}

int main( int argc, char** argv) {

  ros::init(argc, argv, "base_interface");
  
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("base/cmd_vel", 1, cmd_velCallback);
  
  ros::Publisher pub_x = n.advertise<std_msgs::Float64>("base/x_position_controller/command", 50);
  ros::Publisher pub_y = n.advertise<std_msgs::Float64>("base/y_position_controller/command", 50);
  ros::Publisher pub_yaw = n.advertise<std_msgs::Float64>("base/yaw_position_controller/command", 50);
  
  
  
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  
  
  ros::Rate r(100.0);
  while (n.ok()) {
    
      current_time = ros::Time::now();        

      double dt = (current_time - last_time).toSec();
      double delta_x = (vx * cos(yaw) - vy * sin(yaw)) * dt;
      double delta_y = (vx * sin(yaw) + vy * cos(yaw)) * dt;
      double delta_yaw = omegaz * dt;

      x += delta_x ;
      y += delta_y ;
      yaw += delta_yaw;    
    
    
      std_msgs::Float64 x_msg;
      std_msgs::Float64 y_msg;
      std_msgs::Float64 yaw_msg;
      
      x_msg.data = x;
      y_msg.data = y;
      yaw_msg.data = yaw;
      
      pub_x.publish(x_msg);
      pub_y.publish(y_msg);
      pub_yaw.publish(yaw_msg);


      last_time = current_time;

      ros::spinOnce();
      r.sleep();
    
   }  
}
