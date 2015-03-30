#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf/transform_listener.h>


ros::ServiceClient getModelState_client;
ros::ServiceClient setModelState_client;
ros::ServiceServer closeGripper_server;
ros::ServiceServer openGripper_server;

ros::Publisher pub_finger1;
ros::Publisher pub_finger2;
tf::Transform cylinder_transform;
tf::StampedTransform gripper_transform;
tf::Transform gripper_to_object_transform;

bool gripper_is_closed = false;

double cylinder_length;
double cylinder_radius;

double gripper_space = 0.10;
double gripper_finger_length = 0.18;
double gripper_finger_height = 0.02;
double gripper_finger_offset = 0.08;

void moveGripper(double space){
  
  std_msgs::Float64 msg;
  msg.data = space/2.0;
  
  pub_finger1.publish(msg);
  pub_finger2.publish(msg);  
  
}

bool checkCylinderPosition(){
     
  bool isvalid = false;  
   
  tf::Transform offset_transform;
  tf::Transform axis_transform;
  tf::Transform gripper_cylinder_transform;
  tf::Quaternion rotation;
  rotation.setRPY(0,0,0);
  offset_transform.setRotation(rotation);
  
  for(double i = -cylinder_length/2.0; i < cylinder_length/2.0; i+=0.001){
      offset_transform.setOrigin(tf::Vector3(0, 0, i));
      axis_transform.mult(cylinder_transform, offset_transform); 
      gripper_cylinder_transform.mult(gripper_transform.inverse(), axis_transform);
      
      double x = gripper_cylinder_transform.getOrigin().getX();
      double y = gripper_cylinder_transform.getOrigin().getY();
      double z = gripper_cylinder_transform.getOrigin().getZ();
    
      
      if(!isvalid){
        isvalid = fabs(x) < gripper_finger_height;
        isvalid = isvalid && fabs(y) < gripper_space/2.0;
        isvalid = isvalid && z < (gripper_finger_length + gripper_finger_offset);
        ROS_INFO("Object between fingers");
      } else {
        ROS_INFO("Object not between fingers");
      }
         
  }
    
  return isvalid;
}

int getCylinderState(){
  gazebo_msgs::GetModelState srv;
  srv.request.model_name = "cylinder";
  
  getModelState_client.call(srv);
  
  tf::Vector3 origin;
  tf::Quaternion orientation;
  
  //error checking
  
  origin.setX(srv.response.pose.position.x);
  origin.setY(srv.response.pose.position.y);
  origin.setZ(srv.response.pose.position.z);
  
  orientation.setX(srv.response.pose.orientation.x);
  orientation.setY(srv.response.pose.orientation.y);
  orientation.setZ(srv.response.pose.orientation.z);
  orientation.setW(srv.response.pose.orientation.w);  
  
  cylinder_transform.setOrigin(origin);
  cylinder_transform.setRotation(orientation);
  
  if(checkCylinderPosition()){  
    gripper_to_object_transform.mult(gripper_transform.inverse(), cylinder_transform);
    tf::Vector3 origin = gripper_to_object_transform.getOrigin();
    origin.setY(0);
    gripper_to_object_transform.setOrigin(origin);
    return true;
  }
  
  return false;
  
}

void setCylinderState(){
  
  gazebo_msgs::SetModelState srv; 
  
  tf::Transform new_cylinder_transform;
  
  new_cylinder_transform.mult(gripper_transform, gripper_to_object_transform); 
    
  srv.request.model_state.model_name = "cylinder";
  srv.request.model_state.pose.position.x = new_cylinder_transform.getOrigin().getX();
  srv.request.model_state.pose.position.y = new_cylinder_transform.getOrigin().getY();
  srv.request.model_state.pose.position.z = new_cylinder_transform.getOrigin().getZ();
  
  srv.request.model_state.pose.orientation.x = new_cylinder_transform.getRotation().getX();
  srv.request.model_state.pose.orientation.y = new_cylinder_transform.getRotation().getY();
  srv.request.model_state.pose.orientation.z = new_cylinder_transform.getRotation().getZ();
  srv.request.model_state.pose.orientation.w = new_cylinder_transform.getRotation().getW();
  
  setModelState_client.call(srv);  
  
}

bool closeGripperCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
  if(getCylinderState() && !gripper_is_closed){
    gripper_is_closed = true; 
    moveGripper(cylinder_radius*2.1);    
  }
  return true;
}

bool openGripperCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){  
  gripper_is_closed = false; 
  moveGripper(0.08); 
  return true;
}


int main( int argc, char** argv) {

  ros::init(argc, argv, "gripper_interface");
  
  ros::NodeHandle n;  
  
  ros::service::waitForService("gazebo/get_model_state");
  
  getModelState_client = n.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
  setModelState_client = n.serviceClient<gazebo_msgs::SetModelState>("gazebo/set_model_state");
  
  closeGripper_server = n.advertiseService("omnirob_robin/gripper/close_srv", closeGripperCallback);
  openGripper_server = n.advertiseService("omnirob_robin/gripper/open_srv", openGripperCallback);
  
  pub_finger1 = n.advertise<std_msgs::Float64>("omnirob_robin/gripper/left_position_controller/command", 50);
  pub_finger2 = n.advertise<std_msgs::Float64>("omnirob_robin/gripper/right_position_controller/command", 50);
  
  tf::TransformListener listener;
  
  ros::param::get("objects/cylinder/length", cylinder_length);
  ros::param::get("objects/cylinder/radius", cylinder_radius);
  
  ros::Rate r(200.0);
  while (n.ok()) {
      
      
      try{
         listener.lookupTransform("/world", "gripper/palm_link", ros::Time(0), gripper_transform);
      }  catch (tf::TransformException ex){
          
      }      
      
      if(gripper_is_closed){
        setCylinderState();
      }

      ros::spinOnce();
      r.sleep();
    
   }  
}
