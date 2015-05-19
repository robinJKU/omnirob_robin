#include <ros/ros.h>
#include <sstream> 
#include <fstream>

#include <Eigen/Dense>

//ros includes
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"
#include <eigen_conversions/eigen_msg.h>

//services und messages
#include <std_srvs/Empty.h>

unsigned int nr_of_samples_received=0;
std::vector<geometry_msgs::Pose> marker0_pose;
std::vector<geometry_msgs::Pose> marker1_pose;
std::vector<geometry_msgs::Pose> marker2_pose;
const unsigned int marker0_id=0;
const unsigned int marker1_id=1;
const unsigned int marker2_id=2;
std::string marker0_frame_id;
std::string marker1_frame_id;
std::string marker2_frame_id;

void ar_pose_marker_callback( ar_track_alvar_msgs::AlvarMarkers marker_msg ){
	for(unsigned int marker_ii=0; marker_ii<marker_msg.markers.size(); marker_ii++ ){
		switch( marker_msg.markers[marker_ii].id ){
			case marker0_id:
				marker0_pose.push_back( marker_msg.markers[marker_ii].pose.pose);
				if( marker0_pose.size()==1 ){
					marker0_frame_id = marker_msg.markers[marker_ii].header.frame_id;
				}
			break;
			case marker1_id:
				marker1_pose.push_back( marker_msg.markers[marker_ii].pose.pose);
				if( marker1_pose.size()==1 ){
					marker1_frame_id = marker_msg.markers[marker_ii].header.frame_id;
				}
			break;
			case marker2_id:
				marker2_pose.push_back( marker_msg.markers[marker_ii].pose.pose);
				if( marker2_pose.size()==1 ){
					marker2_frame_id = marker_msg.markers[marker_ii].header.frame_id;
				}
			break;
		}
		
	}// for all markeres
	nr_of_samples_received++;
	
}

geometry_msgs::Quaternion calc_quaternion_average( std::vector<geometry_msgs::Pose> pose_vector ){
	
	Eigen::Matrix4f averaging_matrix;
	Eigen::Vector4f quaternion;
	
	averaging_matrix.setZero();
	for( unsigned int sample_ii=0; sample_ii<pose_vector.size(); sample_ii++){
		quaternion(0) = pose_vector[sample_ii].orientation.x;
		quaternion(1) = pose_vector[sample_ii].orientation.y;
		quaternion(2) = pose_vector[sample_ii].orientation.z;
		quaternion(3) = pose_vector[sample_ii].orientation.w;
		averaging_matrix =  averaging_matrix + quaternion*quaternion.transpose();
	  }// for all samples
	 
	 Eigen::EigenSolver<Eigen::Matrix4f> ev_solver;
	 ev_solver.compute( averaging_matrix);
	 Eigen::Vector4cf eigen_values = ev_solver.eigenvalues();
	 float max_ev=eigen_values(0).real();
	 unsigned int max_ii = 0;
	 for( unsigned int ev_ii=1; ev_ii<4; ev_ii++){
		 if( eigen_values(ev_ii).real()>max_ev ){
			 max_ev = eigen_values(ev_ii).real();
			 max_ii=ev_ii;
		 }
	 }
	 
	 Eigen::Vector4f eigen_vector = ev_solver.eigenvectors().col(max_ii).real();
	 eigen_vector.normalize();
	  
	 geometry_msgs::Quaternion quaternion_msg;
	 quaternion_msg.x = eigen_vector(0);
	 quaternion_msg.y = eigen_vector(1);
	 quaternion_msg.z = eigen_vector(2);
	 quaternion_msg.w = eigen_vector(3);
	 return quaternion_msg;
	 
}

geometry_msgs::Point calc_position_average( std::vector<geometry_msgs::Pose> pose_vector ){
	geometry_msgs::Point common_frame_r_marker0;
	common_frame_r_marker0.x=0.0;
	common_frame_r_marker0.y=0.0;
	common_frame_r_marker0.z=0.0;
	  for( unsigned int sample_ii=0; sample_ii<pose_vector.size(); sample_ii++){
		  common_frame_r_marker0.x += pose_vector[sample_ii].position.x;
		  common_frame_r_marker0.y += pose_vector[sample_ii].position.y;
		  common_frame_r_marker0.z += pose_vector[sample_ii].position.z;
	  }// for all samples
	  common_frame_r_marker0.x = common_frame_r_marker0.x/pose_vector.size();
	  common_frame_r_marker0.y = common_frame_r_marker0.y/pose_vector.size();
	  common_frame_r_marker0.z = common_frame_r_marker0.z/pose_vector.size();
	  
	  return common_frame_r_marker0;
  }
  
 inline double sqr( const double value ){
	 return value*value;
 }
  
geometry_msgs::Point calc_std_deviation( std::vector<geometry_msgs::Pose> pose_vector){
	geometry_msgs::Point mean = calc_position_average( pose_vector);
	 
	geometry_msgs::Point common_frame_r_marker0;
	common_frame_r_marker0.x=0.0;
	common_frame_r_marker0.y=0.0;
	common_frame_r_marker0.z=0.0;
	  for( unsigned int sample_ii=0; sample_ii<pose_vector.size(); sample_ii++){
		  common_frame_r_marker0.x += sqr(pose_vector[sample_ii].position.x-mean.x);
		  common_frame_r_marker0.y += sqr(pose_vector[sample_ii].position.y-mean.y);
		  common_frame_r_marker0.z += sqr(pose_vector[sample_ii].position.z-mean.z);
	  }// for all samples
	  common_frame_r_marker0.x = sqrt( common_frame_r_marker0.x/pose_vector.size());
	  common_frame_r_marker0.y = sqrt( common_frame_r_marker0.y/pose_vector.size());
	  common_frame_r_marker0.z = sqrt( common_frame_r_marker0.z/pose_vector.size());
	  
	  return common_frame_r_marker0;
  }
 

geometry_msgs::Pose calc_pose_average( std::vector<geometry_msgs::Pose> pose_vector ){
	geometry_msgs::Pose avg;
	avg.position = calc_position_average( pose_vector);
	avg.orientation = calc_quaternion_average( pose_vector);
	
	return avg;
}

 tf::Transform calc_pose_average( std::vector<tf::Transform> pose_vector ){
	std::vector<geometry_msgs::Pose> temp_pose_vector;
	for( unsigned int pose_ii=0; pose_ii<pose_vector.size(); pose_ii++){
		geometry_msgs::Pose temp_pose;
		tf::poseTFToMsg( pose_vector[pose_ii], temp_pose);
		temp_pose_vector.push_back(temp_pose);
	}
	
	geometry_msgs::Pose temp_pose_msg = calc_pose_average( temp_pose_vector);
	tf::Transform avg_pose;
	tf::poseMsgToTF( temp_pose_msg, avg_pose);
	return avg_pose;
	
}
  
void print_statistic_attributes( std::vector<geometry_msgs::Pose> pose_vector ){

	  geometry_msgs::Point common_frame_r_marker0 = calc_position_average( pose_vector);
	  geometry_msgs::Point std_dev_marker0 = calc_std_deviation( pose_vector);
	  
	  printf("marker0:\n");
	  printf("  x = %f (%f)\n",common_frame_r_marker0.x, std_dev_marker0.x);
	  printf("  y = %f (%f)\n",common_frame_r_marker0.y, std_dev_marker0.y);
	  printf("  z = %f (%f)\n",common_frame_r_marker0.z, std_dev_marker0.z);
	  printf("\n");
}

int main( int argc, char** argv) {

  // initialize node
  ros::init(argc, argv, "marker_calibration");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_ns_node_handle("~");
  
  unsigned int nr_of_samples=50;
  
  // subscribe to topic
  ros::Subscriber ar_pose_marker_subscriber = node_handle.subscribe("/ar_pose_marker", 10, ar_pose_marker_callback);

  // wait for 
  ros::Rate sleep_rate(1.0);
  while( ros::ok() && nr_of_samples_received<nr_of_samples ){
	  ros::spinOnce();
	   sleep_rate.sleep();
	   printf("got %i samples\n",nr_of_samples_received);
  }
  
  if( nr_of_samples<=nr_of_samples_received ){
	  printf("received enough markers\n");
	  printf("marker_0 = %i, marker_1 = %i, marker_2 = %i\n", (int) marker0_pose.size(),  (int) marker1_pose.size(), (int) marker2_pose.size());
	  ROS_INFO("average result");
	  print_statistic_attributes( marker0_pose );
	  print_statistic_attributes( marker1_pose );
	  print_statistic_attributes( marker2_pose );
	  
	  geometry_msgs::Pose marker0_pose_avg;
	  marker0_pose_avg.position = calc_position_average( marker0_pose );
	  marker0_pose_avg.orientation = calc_quaternion_average( marker0_pose );
	  
	  geometry_msgs::Pose marker1_pose_avg;
	  marker1_pose_avg.position = calc_position_average( marker1_pose );
	  marker1_pose_avg.orientation = calc_quaternion_average( marker1_pose );
	  
	  geometry_msgs::Pose marker2_pose_avg;
	  marker2_pose_avg.position = calc_position_average( marker2_pose );
	  marker2_pose_avg.orientation = calc_quaternion_average( marker2_pose );
	  
	  if( marker0_frame_id.compare(marker1_frame_id)!=0 || marker1_frame_id.compare(marker2_frame_id)!=0 ){
		  ROS_ERROR("no common axis found");
		  return -1;
	  }
	  
	  tf::Transform marker0_to_common_axis, marker1_to_common_axis, marker2_to_common_axis;
	  tf::Transform marker0_to_map, marker1_to_map, marker2_to_map;
	  
	  tf::poseMsgToTF( marker0_pose_avg, marker0_to_common_axis);
	  tf::poseMsgToTF( marker1_pose_avg, marker1_to_common_axis);
	  tf::poseMsgToTF( marker2_pose_avg, marker2_to_common_axis);
	  
	  // calc transformation marker0_to_map
	  marker0_to_map.getOrigin()[0] = (marker1_to_common_axis.inverse()*marker0_to_common_axis.getOrigin())[2];
	  marker0_to_map.getOrigin()[1] = 0.0;
	  marker0_to_map.getOrigin()[2] = (marker2_to_common_axis.inverse()*marker0_to_common_axis.getOrigin())[2];	  
	  marker0_to_map.setBasis( tf::Matrix3x3(1, 0, 0,  0, 0, 1,  0,-1, 0));
	  
	  // calc transformation marker1_to_map
	  marker1_to_map.getOrigin()[0] = 0.0;
	  marker1_to_map.getOrigin()[1] = (marker0_to_common_axis.inverse()*marker1_to_common_axis.getOrigin())[2];
	  marker1_to_map.getOrigin()[2] = (marker2_to_common_axis.inverse()*marker1_to_common_axis.getOrigin())[2];
	  marker1_to_map.setBasis( tf::Matrix3x3(0, 0, 1,  1, 0, 0,  0, 1, 0));
	  
	  // calc transformation marker1_to_map
	  marker2_to_map.getOrigin()[0] = (marker1_to_common_axis.inverse()*marker2_to_common_axis.getOrigin())[2];
	  marker2_to_map.getOrigin()[1] = (marker0_to_common_axis.inverse()*marker2_to_common_axis.getOrigin())[2];
	  marker2_to_map.getOrigin()[2] = 0.0;
	  marker2_to_map.setBasis( tf::Matrix3x3(1, 0, 0,  0, 1, 0,  0, 0, 1));
	  
	  tf::Transform map_to_common_axis0, map_to_common_axis1, map_to_common_axis2;
	  map_to_common_axis0 = marker0_to_common_axis*marker0_to_map.inverse();
	  map_to_common_axis1 = marker1_to_common_axis*marker1_to_map.inverse();
	  map_to_common_axis2 = marker2_to_common_axis*marker2_to_map.inverse();
	  
	  printf("marker 2 2 common axis = [%f,%f,%f]\n", marker2_to_common_axis.getOrigin()[0], marker2_to_common_axis.getOrigin()[1], marker2_to_common_axis.getOrigin()[2]);
	  printf("marker 2 2 map = [%f,%f,%f]\n", marker2_to_map.getOrigin()[0], marker2_to_map.getOrigin()[1], marker2_to_map.getOrigin()[2]);
	  printf("map 2 marker 2 = [%f,%f,%f]\n", marker2_to_map.inverse().getOrigin()[0], marker2_to_map.inverse().getOrigin()[1], marker2_to_map.inverse().getOrigin()[2]);
	  printf("map 2 common axis 2 = [%f,%f,%f]\n", map_to_common_axis2.getOrigin()[0], map_to_common_axis2.getOrigin()[1], map_to_common_axis2.getOrigin()[2]);
	  
	  
	  std::vector<tf::Transform> map_to_common_axis_vector;
	  map_to_common_axis_vector.push_back( map_to_common_axis0);
	  map_to_common_axis_vector.push_back( map_to_common_axis1);
	  map_to_common_axis_vector.push_back( map_to_common_axis2);
	  
	  tf::Transform map_to_common_axis_avg = calc_pose_average( map_to_common_axis_vector);
	  
	  static tf::TransformBroadcaster br;
	  
	  printf("map 2 common axis = [%f,%f,%f]\n", map_to_common_axis_avg.getOrigin()[0], map_to_common_axis_avg.getOrigin()[1], map_to_common_axis_avg.getOrigin()[2]);
	  printf("map 2 common axis avg = [%f,%f,%f]\n", map_to_common_axis_avg.getOrigin()[0], map_to_common_axis_avg.getOrigin()[1], map_to_common_axis_avg.getOrigin()[2]);
	  tf::Transform marker0_to_map_avg =  map_to_common_axis_avg.inverse()*marker0_to_common_axis;
	tf::Transform marker1_to_map_avg =  map_to_common_axis_avg.inverse()*marker1_to_common_axis;
	tf::Transform marker2_to_map_avg =  map_to_common_axis_avg.inverse()*marker2_to_common_axis;
	
	ROS_INFO("save parameters");
	std::ofstream yaml_file( "/home/omnirob/catkin_ws/src/omnirob_robin/omnirob_robin_object_detection/data/marker_calibration.yaml" );
	yaml_file << "marker_ids: [\'marker0\', \'marker1\', \'marker2\']\n";
	yaml_file << "marker0:\n"; // <!-- described in map coordinates -->
	yaml_file << "  position:\n";
	yaml_file << "    x: " << marker0_to_map_avg.getOrigin()[0] <<"\n";
	yaml_file << "    y: " << marker0_to_map_avg.getOrigin()[1] <<"\n";
	yaml_file << "    z: " << marker0_to_map_avg.getOrigin()[2] <<"\n";
	yaml_file << "  orientation:\n";
	yaml_file << "    x: " << marker0_to_map_avg.getRotation()[0] <<"\n";
	yaml_file << "    y: " << marker0_to_map_avg.getRotation()[1] <<"\n";
	yaml_file << "    z: " << marker0_to_map_avg.getRotation()[2] <<"\n";
	yaml_file << "    w: " << marker0_to_map_avg.getRotation()[3] <<"\n";
	yaml_file << "marker1:\n"; // <!-- described in map coordinates -->
	yaml_file << "  position:\n";
	yaml_file << "    x: " << marker1_to_map_avg.getOrigin()[0] <<"\n";
	yaml_file << "    y: " << marker1_to_map_avg.getOrigin()[1] <<"\n";
	yaml_file << "    z: " << marker1_to_map_avg.getOrigin()[2] <<"\n";
	yaml_file << "  orientation:\n";
	yaml_file << "    x: " << marker1_to_map_avg.getRotation()[0] <<"\n";
	yaml_file << "    y: " << marker1_to_map_avg.getRotation()[1] <<"\n";
	yaml_file << "    z: " << marker1_to_map_avg.getRotation()[2] <<"\n";
	yaml_file << "    w: " << marker1_to_map_avg.getRotation()[3] <<"\n";
	yaml_file << "marker2:\n"; // <!-- described in map coordinates -->
	yaml_file << "  position:\n";
	yaml_file << "    x: " << marker2_to_map_avg.getOrigin()[0] <<"\n";
	yaml_file << "    y: " << marker2_to_map_avg.getOrigin()[1] <<"\n";
	yaml_file << "    z: " << marker2_to_map_avg.getOrigin()[2] <<"\n";
	yaml_file << "  orientation:\n";
	yaml_file << "    x: " << marker2_to_map_avg.getRotation()[0] <<"\n";
	yaml_file << "    y: " << marker2_to_map_avg.getRotation()[1] <<"\n";
	yaml_file << "    z: " << marker2_to_map_avg.getRotation()[2] <<"\n";
	yaml_file << "    w: " << marker2_to_map_avg.getRotation()[3] <<"\n";
	yaml_file.close();
	  
	  while( ros::ok() ){
		br.sendTransform(tf::StampedTransform(marker0_to_common_axis, ros::Time::now(), marker0_frame_id, "marker0_avg"));
		br.sendTransform(tf::StampedTransform(marker1_to_common_axis, ros::Time::now(), marker1_frame_id, "marker1_avg"));
		br.sendTransform(tf::StampedTransform(marker2_to_common_axis, ros::Time::now(), marker2_frame_id, "marker2_avg"));
		
		br.sendTransform(tf::StampedTransform(map_to_common_axis0, ros::Time::now(), marker0_frame_id, "init_frame_0"));
		br.sendTransform(tf::StampedTransform(map_to_common_axis1, ros::Time::now(), marker1_frame_id, "init_frame_1"));
		br.sendTransform(tf::StampedTransform(map_to_common_axis2, ros::Time::now(), marker2_frame_id, "init_frame_2"));
		
		br.sendTransform(tf::StampedTransform(map_to_common_axis_avg, ros::Time::now(), marker2_frame_id, "init_frame_avg"));
		
		sleep_rate.sleep();
	}
	  
  }
  
}
