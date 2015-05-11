#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <controller_interface/controller.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>


joint_limits_interface::JointLimits limits_lwa1;
joint_limits_interface::JointLimits limits_lwa2;
joint_limits_interface::JointLimits limits_lwa3;
joint_limits_interface::JointLimits limits_lwa4;
joint_limits_interface::JointLimits limits_lwa5;
joint_limits_interface::JointLimits limits_lwa6;
joint_limits_interface::JointLimits limits_lwa7;
joint_limits_interface::SoftJointLimits soft_limits_lwa1;
joint_limits_interface::SoftJointLimits soft_limits_lwa2;
joint_limits_interface::SoftJointLimits soft_limits_lwa3;
joint_limits_interface::SoftJointLimits soft_limits_lwa4;
joint_limits_interface::SoftJointLimits soft_limits_lwa5;
joint_limits_interface::SoftJointLimits soft_limits_lwa6;
joint_limits_interface::SoftJointLimits soft_limits_lwa7;

double vel2;

class MyRobot : public hardware_interface::RobotHW
{
public:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  joint_limits_interface::PositionJointSoftLimitsInterface jnt_limits_interface_;
  double cmd[7];
  double pos[7];
  double vel[7];
  double eff[7];

  MyRobot() {}
  
  bool init()
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

    // Register handle in joint limits interface
    joint_limits_interface::PositionJointSoftLimitsHandle handle_lwa1(pos_handle_lwa1,
                                         limits_lwa1,       // Limits spec
                                         soft_limits_lwa1);  // Soft limits spec
    joint_limits_interface::PositionJointSoftLimitsHandle handle_lwa2(pos_handle_lwa2,
                                         limits_lwa2,       // Limits spec
                                         soft_limits_lwa2);  // Soft limits spec
    joint_limits_interface::PositionJointSoftLimitsHandle handle_lwa3(pos_handle_lwa3,
                                         limits_lwa3,       // Limits spec
                                         soft_limits_lwa3);  // Soft limits spec
    joint_limits_interface::PositionJointSoftLimitsHandle handle_lwa4(pos_handle_lwa4,
                                         limits_lwa4,       // Limits spec
                                         soft_limits_lwa4);  // Soft limits spec
    joint_limits_interface::PositionJointSoftLimitsHandle handle_lwa5(pos_handle_lwa5,
                                         limits_lwa5,       // Limits spec
                                         soft_limits_lwa5);  // Soft limits spec
    joint_limits_interface::PositionJointSoftLimitsHandle handle_lwa6(pos_handle_lwa6,
                                         limits_lwa6,       // Limits spec
                                         soft_limits_lwa6);  // Soft limits spec
    joint_limits_interface::PositionJointSoftLimitsHandle handle_lwa7(pos_handle_lwa7,
                                         limits_lwa7,       // Limits spec
                                         soft_limits_lwa7);  // Soft limits spec

    //joint_limits_interface::VelocityJointSaturationHandle handle_lwa2(pos_handle_lwa2,
    //                                     limits_lwa2);

    jnt_limits_interface_.registerHandle(handle_lwa1);
    jnt_limits_interface_.registerHandle(handle_lwa2);
    jnt_limits_interface_.registerHandle(handle_lwa3);
    jnt_limits_interface_.registerHandle(handle_lwa4);
    jnt_limits_interface_.registerHandle(handle_lwa5);
    jnt_limits_interface_.registerHandle(handle_lwa6);
    jnt_limits_interface_.registerHandle(handle_lwa7);
//jnt_limits_interface_.registerHandle(handle_lwa2);
    
  }

  void write(ros::Publisher trajectoryPoint_pub, std_msgs::Float64MultiArray trajPoint, ros::Duration period) {
     jnt_limits_interface_.enforceLimits(period);
     // Publish trajectory message
     trajPoint.data.resize(7);
     for(int i=0; i<7; i++) { 
     	trajPoint.data[i] = cmd[i];
     }
     //trajPoint.data[0] = cmd[1];
     //trajPoint.data[1] = (cmd[1]-vel2)/0.01;
     trajectoryPoint_pub.publish(trajPoint);
     ROS_INFO("publishing commanded position: %f",cmd[0]);
     vel2 = cmd[1];
     return;
  }

};

MyRobot omniRob;

void jointState_callback(const sensor_msgs::JointState msg)
{
  for(int i=0; i<7; i++) { 
  	omniRob.pos[i]=msg.position[i];
  	omniRob.vel[i]=msg.velocity[i];
  	omniRob.eff[i]=msg.effort[i];
  }
}

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "omnirob_robin_lwa_interface");

  ros::NodeHandle n;
  ros::Rate loop_rate(100);	// 100 Hz => consistent to sampleTime
  ros::AsyncSpinner spinner(4); // Use 4 threads
  // ...initialize contents of urdf
  //boost::shared_ptr<urdf::ModelInterface> urdf;
  vel2 = 0.0;
  controller_manager::ControllerManager cm(&omniRob);
  ros::Duration sampleTime(0.01);
  ros::Publisher trajectoryPoint_pub = n.advertise<std_msgs::Float64MultiArray>("lwa/control/commanded_joint_state", 1000);
  ros::Subscriber jointState_sub = n.subscribe("lwa/state/joint_state", 1000, jointState_callback);

  std_msgs::Float64MultiArray trajPoint;

  spinner.start();

  // ----
  // Data structures
  // Manual value setting
  limits_lwa1.has_velocity_limits = true;
  limits_lwa1.max_velocity = 0.2;
  limits_lwa2.has_velocity_limits = true;
  limits_lwa2.max_velocity = 0.2;
  limits_lwa3.has_velocity_limits = true;
  limits_lwa3.max_velocity = 0.2;
  limits_lwa4.has_velocity_limits = true;
  limits_lwa4.max_velocity = 0.2;
  limits_lwa5.has_velocity_limits = true;
  limits_lwa5.max_velocity = 0.2;
  limits_lwa6.has_velocity_limits = true;
  limits_lwa6.max_velocity = 0.2;
  limits_lwa7.has_velocity_limits = true;
  limits_lwa7.max_velocity = 0.2;
  
  limits_lwa1.has_acceleration_limits = true;
  limits_lwa1.max_acceleration = 1.0;
  limits_lwa2.has_acceleration_limits = true;
  limits_lwa2.max_acceleration = 1.0;
  limits_lwa3.has_acceleration_limits = true;
  limits_lwa3.max_acceleration = 1.0;
  limits_lwa4.has_acceleration_limits = true;
  limits_lwa4.max_acceleration = 1.0;
  limits_lwa5.has_acceleration_limits = true;
  limits_lwa5.max_acceleration = 1.0;
  limits_lwa6.has_acceleration_limits = true;
  limits_lwa6.max_acceleration = 1.0;
  limits_lwa7.has_acceleration_limits = true;
  limits_lwa7.max_acceleration = 1.0;

  // Populate (soft) joint limits from URDF
  // Limits specified in URDF overwrite existing values in 'limits' and 'soft_limits'
  // Limits not specified in URDF preserve their existing values
  //boost::shared_ptr<const urdf::Joint> urdf_lwa_joint1 = urdf->getJoint("lwa/joint_1");
  //const bool urdf_limits_ok = getJointLimits(urdf_lwa_joint1, limits);
  //const bool urdf_soft_limits_ok = getSoftJointLimits(urdf_lwa_joint1, soft_limits);

  // Populate (soft) joint limits from the ros parameter server
  // Limits specified in the parameter server overwrite existing values in 'limits' and 'soft_limits'
  // Limits not specified in the parameter server preserve their existing values
  //const bool rosparam_limits_ok = getJointLimits("joint_1", n, limits_lwa1);
  // ----
  
  omniRob.init();

  ROS_INFO("Max velocity: %f",limits_lwa2.max_velocity);
  while (ros::ok())
  {
     //omniRob.read();  // automated subscription
     cm.update(ros::Time::now(), sampleTime);
     omniRob.write(trajectoryPoint_pub, trajPoint, sampleTime);
     ROS_INFO("omnirob LWA3 OK...");

     //ros::spinOnce();
     //ros::waitForShutdown();
     loop_rate.sleep();
     //spinner.stop();
  }
  return 0;  
}

