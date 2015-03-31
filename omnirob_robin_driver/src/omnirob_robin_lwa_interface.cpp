#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <controller_interface/controller.h>

class MyRobot : public hardware_interface::RobotHW
{
public:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[7];
  double pos[7];
  double vel[7];
  double eff[7];

  MyRobot() 
 { 
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_lwa1("lwa/joint_1", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_lwa1);

   hardware_interface::JointStateHandle state_handle_lwa2("lwa/joint_2", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_lwa2);

   hardware_interface::JointStateHandle state_handle_lwa3("lwa/joint_3", &pos[2], &vel[2], &eff[2]);
   jnt_state_interface.registerHandle(state_handle_lwa3);

   hardware_interface::JointStateHandle state_handle_lwa4("lwa/joint_4", &pos[3], &vel[3], &eff[3]);
   jnt_state_interface.registerHandle(state_handle_lwa4);

   hardware_interface::JointStateHandle state_handle_lwa5("lwa/joint_5", &pos[4], &vel[4], &eff[4]);
   jnt_state_interface.registerHandle(state_handle_lwa5);

   hardware_interface::JointStateHandle state_handle_lwa6("lwa/joint_6", &pos[5], &vel[5], &eff[5]);
   jnt_state_interface.registerHandle(state_handle_lwa6);

   hardware_interface::JointStateHandle state_handle_lwa7("lwa/joint_7", &pos[6], &vel[6], &eff[6]);
   jnt_state_interface.registerHandle(state_handle_lwa7);   

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle pos_handle_lwa1(jnt_state_interface.getHandle("lwa/joint_1"), &cmd[0]);
   jnt_pos_interface.registerHandle(pos_handle_lwa1);

   hardware_interface::JointHandle pos_handle_lwa2(jnt_state_interface.getHandle("lwa/joint_2"), &cmd[1]);
   jnt_pos_interface.registerHandle(pos_handle_lwa2);

   hardware_interface::JointHandle pos_handle_lwa3(jnt_state_interface.getHandle("lwa/joint_3"), &cmd[2]);
   jnt_pos_interface.registerHandle(pos_handle_lwa3);

   hardware_interface::JointHandle pos_handle_lwa4(jnt_state_interface.getHandle("lwa/joint_4"), &cmd[3]);
   jnt_pos_interface.registerHandle(pos_handle_lwa4);

   hardware_interface::JointHandle pos_handle_lwa5(jnt_state_interface.getHandle("lwa/joint_5"), &cmd[4]);
   jnt_pos_interface.registerHandle(pos_handle_lwa5);

   hardware_interface::JointHandle pos_handle_lwa6(jnt_state_interface.getHandle("lwa/joint_6"), &cmd[5]);
   jnt_pos_interface.registerHandle(pos_handle_lwa6);

   hardware_interface::JointHandle pos_handle_lwa7(jnt_state_interface.getHandle("lwa/joint_7"), &cmd[6]);
   jnt_pos_interface.registerHandle(pos_handle_lwa7);

   registerInterface(&jnt_pos_interface);

  }
  
  void write(ros::Publisher trajectoryPoint_pub, std_msgs::Float64MultiArray trajPoint) {
     // Publish trajectory message
     trajPoint.data.resize(7);
     for(int i=0; i<7; i++) { 
     	trajPoint.data[i] = cmd[i];
     }
     trajectoryPoint_pub.publish(trajPoint);
     ROS_INFO("publishing commanded position: %f",cmd[0]);
     return;
  }

/*private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[7];
  double pos[7];
  double vel[7];
  double eff[7];*/
};

MyRobot omniRob;

void jointState_callback(const sensor_msgs::JointState msg)
{
  for(int i=0; i<7; i++) { 
  	omniRob.pos[i]=msg.position[i];
  	omniRob.vel[i]=msg.velocity[i];
  	omniRob.eff[i]=msg.effort[i];
  }
  //ROS_INFO("I heard something...");
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "omnirob_robin_lwa_interface");

  ros::NodeHandle n;
  ros::Rate loop_rate(100);	// 100 Hz => consistent to sampleTime
  ros::AsyncSpinner spinner(4); // Use 4 threads

  //MyRobot omniRob;
  controller_manager::ControllerManager cm(&omniRob);
  ros::Duration sampleTime(0.01);
  ros::Publisher trajectoryPoint_pub = n.advertise<std_msgs::Float64MultiArray>("lwa/control/commanded_joint_state", 1000);
  ros::Subscriber jointState_sub = n.subscribe("lwa/state/joint_state", 1000, jointState_callback);

  std_msgs::Float64MultiArray trajPoint;

  spinner.start();

  while (ros::ok())
  {
     //omniRob.read();  // automated subscription
     cm.update(ros::Time::now(), sampleTime);
     omniRob.write(trajectoryPoint_pub, trajPoint);
     ROS_INFO("omnirob LWA3 OK...");

     //ros::spinOnce();
     //ros::waitForShutdown();
     loop_rate.sleep();
     //spinner.stop();
  }
  return 0;
}

