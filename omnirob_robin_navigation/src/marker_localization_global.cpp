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
#include <dynamic_reconfigure/Reconfigure.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64MultiArray.h>

// global variables
unsigned int nr_of_samples_received;
bool detection_mode;
tf::TransformListener *transform_listener;

// prototypes
std::string unsigned_int_to_string( unsigned int value );

class Pose_Samples{
	public:
	std::vector<geometry_msgs::Pose> samples;
	
	/**
	 * Calculates an average of the specified orientations based on the minimization of the error between
	 * the rotation matrix computed from the average orientation and the rotaton matrices computed with the
	 * specified orientations, whereby error is measured with the Frobenius norm.
	 * The algorithm is proposed in the paper "Quaternion Averaging" by F. Landis Markley, Yang Cheng,
	 * John L. Crassidis and Yaakov Oshman. 
	 */
	geometry_msgs::Quaternion calc_quaternion_average( void ){
		Eigen::Matrix4f averaging_matrix;
		Eigen::Vector4f quaternion;
		
		averaging_matrix.setZero();
		for( unsigned int sample_ii=0; sample_ii<samples.size(); sample_ii++){
			quaternion(0) = samples[sample_ii].orientation.x;
			quaternion(1) = samples[sample_ii].orientation.y;
			quaternion(2) = samples[sample_ii].orientation.z;
			quaternion(3) = samples[sample_ii].orientation.w;
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
	}// calc orientation average
	
	/**
	 * Calculates an average of the specified positions. The average is taken component by component and the arithmetic
	 * average is used.
	 */
	geometry_msgs::Point calc_position_average( void ){
		geometry_msgs::Point common_frame_r_marker0;
		common_frame_r_marker0.x=0.0;
		common_frame_r_marker0.y=0.0;
		common_frame_r_marker0.z=0.0;
		for( unsigned int sample_ii=0; sample_ii<samples.size(); sample_ii++){
			common_frame_r_marker0.x += samples[sample_ii].position.x;
			common_frame_r_marker0.y += samples[sample_ii].position.y;
			common_frame_r_marker0.z += samples[sample_ii].position.z;
		}// for all samples
		common_frame_r_marker0.x = common_frame_r_marker0.x/samples.size();
		common_frame_r_marker0.y = common_frame_r_marker0.y/samples.size();
		common_frame_r_marker0.z = common_frame_r_marker0.z/samples.size();
		
		return common_frame_r_marker0;
	}// calc position average
	
	/**
	 * Computes the orientation average as well as the position average
	 * @see: calc_position_average
	 * @see: calc_quaternion_average
	 */
	geometry_msgs::Pose calc_average( void ){
		geometry_msgs::Pose avg;
		avg.position = this->calc_position_average();
		avg.orientation = this->calc_quaternion_average();
	
		return avg;
	}// calc average
	
	void resize( unsigned int size ){
		samples.resize( size);
	}// resize
	
	void push_back( geometry_msgs::Pose entry ){
		samples.push_back( entry);
	}// push_back
	
	
};// Pose_Samples

class AR_Marker{
	public:
	unsigned int marker_id;
	tf::Transform pose_inertial_frame;
	Pose_Samples observed_pose_reference_frame;
	std::string reference_frame_id;
	bool marker_detected;
	
	private:
	geometry_msgs::Pose avg_marker_pose_reference_frame;
	bool avg_pose_is_initialized;
	
	public:
	/**
	 * default constructor
	 */
	AR_Marker():avg_pose_is_initialized(false){}
	
	public:
	geometry_msgs::Pose avg_pose( void ){
		if( !avg_pose_is_initialized ){
			// avg pose isn't calculated
			avg_marker_pose_reference_frame = observed_pose_reference_frame.calc_average();
			avg_pose_is_initialized = true;
		}
		return avg_marker_pose_reference_frame;
	}
	
	void clear( void ){
		observed_pose_reference_frame.resize(0);
		reference_frame_id.clear();
		avg_pose_is_initialized=false;
		marker_detected=false;
	}// clear
	
	std::string to_string( void ){
		return unsigned_int_to_string( marker_id);
	}// to string
};

class AR_Marker_Localization{
	public:
	std::vector<AR_Marker> mark;
	std::string base_link;
	
	private:
	unsigned int last_considered_id;
	geometry_msgs::Pose estimated_base_pose_inertial_frame;
	bool base_is_localized;
	
	public:
	/**
	 * default constructor
	 */
	AR_Marker_Localization (): base_is_localized(false), last_considered_id(0){}
	
	bool localize_base_frame( geometry_msgs::Pose &estimation ){
		ROS_INFO("localize base link");
		Pose_Samples base_link_poses;
		
		for( unsigned int mark_ii=0; mark_ii<mark.size(); mark_ii++){
			if( mark[mark_ii].marker_detected ){
				// calculate marker pose in reference frame
				tf::Transform pose_mark_ii_refence_frame;
				tf::poseMsgToTF( mark[mark_ii].avg_pose(), pose_mark_ii_refence_frame);
				
				// transformation marker pose to base_link frame
				tf::StampedTransform to_reference_frame_from_base_link;
				if( base_link.empty() ){
					ROS_ERROR("Can't located base link - No base link frame specified");
					return false;
				}
				try{
				  // the listener need time to buffer tf data
				  transform_listener->waitForTransform( mark[mark_ii].reference_frame_id, base_link, ros::Time::now(), ros::Duration(2.0));
				  transform_listener->lookupTransform( mark[mark_ii].reference_frame_id, base_link, ros::Time(0), to_reference_frame_from_base_link);
				}
				catch (tf::TransformException ex){
				  ROS_ERROR("Can't lookup reference frame: %s",ex.what());
				  return false;
				}
				
				// calculate base_link pose in map_frame
				geometry_msgs::Pose base_link_pose_map_frame;
				tf::poseTFToMsg( mark[mark_ii].pose_inertial_frame*pose_mark_ii_refence_frame.inverse()*to_reference_frame_from_base_link, base_link_pose_map_frame);
				base_link_poses.push_back( base_link_pose_map_frame);
			}
		}// for all marks
		
		// average all markers
		ROS_INFO("calc average");
		estimation = base_link_poses.calc_average();
		
		// select relevant values (x,y,yaw)
		ROS_INFO("select relevant values");
		tf::Transform estimation_tf;
		tf::poseMsgToTF( estimation, estimation_tf);
		double roll, pitch, yaw;
		estimation_tf.getBasis().getRPY( roll, pitch, yaw);
		estimation_tf.getBasis().setRPY( 0.0, 0.0, yaw);
		estimation_tf.getOrigin()[2] = 0.0;
		
		tf::poseTFToMsg( estimation_tf, estimation);
		return true;
		
	}// localize base frame
	
	int get_index( unsigned int id ){
		if( mark.size()==0 ){
			return -1;
		}
		
		unsigned int mark_ii=last_considered_id;
		do{
			if( mark[mark_ii].marker_id == id ){
				// found mark
				last_considered_id=mark_ii;
				return mark_ii;
			}
			mark_ii++;
			if(mark_ii>=mark.size()){
				mark_ii-=mark.size();
			}
		}while(mark_ii!=last_considered_id);
		
		return -1;
	}// get index
	
	bool contains( unsigned int marker_id ){
		return this->get_index( marker_id)>=0? true : false;
	}// contains
	
	void insert_pose( unsigned int marker_id, geometry_msgs::Pose marker_pose, std::string reference_frame_id ){
		unsigned int marker_index = get_index( marker_id );
		if( marker_index>=0 ){
			if( !mark[marker_index].marker_detected ){
				mark[marker_index].marker_detected = true;
				mark[marker_index].reference_frame_id = reference_frame_id;
			}else if( mark[marker_index].reference_frame_id.compare(reference_frame_id)!=0 ){
				ROS_ERROR("Received wrong reference frame id from marker %u. Expected= %s , Received= %s", mark[marker_index].marker_id, mark[marker_index].reference_frame_id.c_str(), reference_frame_id.c_str());
				return;
			}
			mark[marker_index].observed_pose_reference_frame.push_back( marker_pose);
		}
	}// insert pose
	
	void set_marker_ids( std::vector<unsigned int> marker_ids ){
		mark.resize( marker_ids.size());
		for( unsigned marker_ii=0; marker_ii< marker_ids.size(); marker_ii++){
			mark[marker_ii].marker_id = marker_ids[marker_ii];
		}
	}// set marker_ids
	
	void clear( void ){
		for( unsigned int mark_ii=0; mark_ii<mark.size(); mark_ii++){
			mark[mark_ii].clear();
		}
		last_considered_id = 0;
		base_is_localized = false;
	}// clear
	
	std::string to_string( void ){
		std::stringstream string_stream;
		string_stream << "total nr of marker" << mark.size() <<"\n";
		for(unsigned int mark_ii=0; mark_ii<mark.size(); mark_ii++){
			string_stream << "  [" << mark_ii << "]: " << mark[mark_ii].to_string() << "\n";
		}
		return string_stream.str();
	}// to string stream
	
} markers;

void ar_pose_marker_callback( ar_track_alvar_msgs::AlvarMarkers marker_msg ){
	int nr_of_valid_markers=0;
	for( unsigned int marker_ii=0; marker_ii<marker_msg.markers.size(); marker_ii++){
		if( markers.contains( marker_msg.markers[marker_ii].id) ){
			nr_of_valid_markers++;
		}
	}
	
	if( nr_of_valid_markers>0 ){
		if( detection_mode ){
			ROS_INFO("Found valid marker - disable detection mode");
			detection_mode = false;
			
		}else{
			for(unsigned int marker_ii=0; marker_ii<marker_msg.markers.size(); marker_ii++ ){
				// if marker id is known, insert marker pose 
				if( markers.contains( marker_msg.markers[marker_ii].id)){
					markers.insert_pose( marker_msg.markers[marker_ii].id, marker_msg.markers[marker_ii].pose.pose, marker_msg.markers[marker_ii].header.frame_id);
				}else{
					ROS_INFO("Got unrecognized (id: %u) marker pose", marker_msg.markers[marker_ii].id);
				}
				
			}// for all markeres
			nr_of_samples_received++;
		}
	}
}// ar pose marker callback


bool wait_for_service( std::string topic ){
  ros::Rate rate_1s(1);
  unsigned int waiting_cnt=0;
  while( waiting_cnt<10 && ros::ok() && !ros::service::exists(topic, false) ){
	  ROS_INFO("Waiting for service %s", topic.c_str());
	  rate_1s.sleep();
  }
  if( waiting_cnt>10 ){
	  ROS_ERROR("Service %s is not available", topic.c_str());
	  return false;
  }
  return true;
	
}// wait for service

std::string unsigned_int_to_string( unsigned int value ){
	std::stringstream value_string_stream;
	value_string_stream << value;
	return value_string_stream.str();
}// unsigned int to string

void clean_up( void ){
	if( transform_listener!=NULL ){
		delete transform_listener;
	}
}// clean up

int main( int argc, char** argv) {
	 
   // initialize node
  ros::init(argc, argv, "marker_localization_global");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_ns_node_handle("~");
  
  transform_listener = new tf::TransformListener(node_handle);
  
  // validate and read required parameters 
  private_ns_node_handle.getParam("base_link", markers.base_link);
  if( markers.base_link.empty() ){
	  markers.base_link = "/base_link";
  }
  
  if( !private_ns_node_handle.hasParam("marker_ids") ){
	  ROS_ERROR("No marker ids specifies!");
	  clean_up();
	  return -1;
  }
  std::vector<int> marker_ids_int;
  private_ns_node_handle.getParam("marker_ids", marker_ids_int);
  
  std::vector<unsigned int> marker_ids;
  for( unsigned int marker_ii=0; marker_ii<marker_ids_int.size(); marker_ii++){
	  if( marker_ids_int[marker_ii]<0 ){
		  ROS_WARN("Received negative marker %i id - discard id", marker_ids_int[marker_ii]);
	  }else{
		  marker_ids.push_back( (unsigned int) marker_ids_int[marker_ii]);
	  }
  }
  if( marker_ids_int.size()==0 ){
	  ROS_ERROR("Invalid number of marker ids");
	  clean_up();
	  return -1;
  }
  
  std::stringstream output_stream;
  output_stream << "Found marker configuration file. Looking for marker with ids: [";
  for( unsigned int marker_ii=0; marker_ii<marker_ids.size(); marker_ii++){
	 output_stream << marker_ids[marker_ii];
	 if( marker_ii<marker_ids.size()-1){
		 output_stream <<", ";
	 }
  }
  output_stream << "]";
  ROS_INFO("%s", output_stream.str().c_str() );
  
  markers.set_marker_ids( marker_ids);
  for( unsigned int marker_ii=0; marker_ii<marker_ids.size(); marker_ii++ ){
	if(    !private_ns_node_handle.hasParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/position/x")
	    || !private_ns_node_handle.hasParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/position/y")
	    || !private_ns_node_handle.hasParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/position/z")
	    || !private_ns_node_handle.hasParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/orientation/x")
	    || !private_ns_node_handle.hasParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/orientation/y")
	    || !private_ns_node_handle.hasParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/orientation/z")
	    || !private_ns_node_handle.hasParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/orientation/w") ){
		ROS_ERROR("Not enought parameters for marker: %s", unsigned_int_to_string(marker_ids[marker_ii]).c_str());
		clean_up();
		return -1;
		
	}else{
		double rx,ry,rz, ox,oy,oz,ow;
		private_ns_node_handle.getParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/position/x",rx);
	    private_ns_node_handle.getParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/position/y",ry);
	    private_ns_node_handle.getParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/position/z",rz);
	    private_ns_node_handle.getParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/orientation/x",ox);
	    private_ns_node_handle.getParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/orientation/y",oy);
	    private_ns_node_handle.getParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/orientation/z",oz);
	    private_ns_node_handle.getParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/orientation/w",ow);
	    
		markers.mark[ marker_ii].pose_inertial_frame.setOrigin( tf::Vector3(rx,ry,rz));
		markers.mark[ marker_ii].pose_inertial_frame.setRotation( tf::Quaternion(ox,oy,oz,ow));
	}
  }
  ROS_INFO("Data read successfully:");
  ROS_INFO("%s", markers.to_string().c_str());
	
  // wait for services
  std::string pan_tilt_start_motion_srv = "/omnirob_robin/pan_tilt/control/start_motion";
  if( !wait_for_service( pan_tilt_start_motion_srv) ){
	  clean_up();
	  return -1;
  }
  std::string ar_track_set_param_srv = "/ar_track_alvar/set_parameters";
  if( !wait_for_service( ar_track_set_param_srv) ){
	  clean_up();
	  return -1;
  }
  
  /* ---------------------------------------------------------- */
  // enable ar track alvar node
  ros::ServiceClient ar_set_parameters_client = node_handle.serviceClient<dynamic_reconfigure::Reconfigure>( ar_track_set_param_srv);
  dynamic_reconfigure::Reconfigure ar_parameters;
  ar_parameters.request.config.bools.resize(1);
  ar_parameters.request.config.bools[0].name="enabled";
  ar_parameters.request.config.bools[0].value=true;
  ar_set_parameters_client.call( ar_parameters);
  
  // subscribe to marker poses
  ros::Subscriber ar_pose_marker_subscriber = node_handle.subscribe("/ar_pose_marker", 50, ar_pose_marker_callback);
  
  // look around, search for pan angle overlooking the maximum number of markers
  ros::ServiceClient pan_start_motion_client = node_handle.serviceClient<std_srvs::Empty>( pan_tilt_start_motion_srv);
  ros::Publisher pan_commanded_joint_state_pubisher = node_handle.advertise<std_msgs::Float64MultiArray>("/omnirob_robin/pan_tilt/control/commanded_joint_state", 5);
  
  
  
  
  std_srvs::Empty empty_srvs;
  std_msgs::Float64MultiArray pan_goal_position_msgs;
  pan_goal_position_msgs.data.resize(2);
  
  detection_mode = true;
  ros::Rate rate_1s(1);
  rate_1s.sleep(); 
  ros::spinOnce();
  
  double min_angle=-0.9, max_angle=0.9, increment_angle=20.0/180.0*M_PI;
  unsigned int half_part_scanned=false;
  bool last_round=false;
  if( detection_mode ){
	  while( ros::ok() ){
		  
		  ROS_INFO("No marker detected, change pan angle");
		  pan_goal_position_msgs.data[0] -= increment_angle;
		  if( pan_goal_position_msgs.data[0]<min_angle ){
			  pan_goal_position_msgs.data[0]=pan_goal_position_msgs.data[1] + max_angle - min_angle;
			  half_part_scanned = true;
		  }
		  
		  if( half_part_scanned && pan_goal_position_msgs.data[0]<(max_angle+min_angle)/2.0 ){
			  ROS_ERROR("Finished scan, no marker detected");
			  clean_up();
			  return -1;
		  }
		  pan_commanded_joint_state_pubisher.publish( pan_goal_position_msgs);
		  ros::spinOnce();
		  pan_start_motion_client.call( empty_srvs);
		  // wait for pose reached
		  rate_1s.sleep();
		  
		  // wait for marker detection
		  rate_1s.sleep(); 
		  ros::spinOnce();
		  
		  if( !detection_mode ){
			  if( (!last_round) && (min_angle<pan_goal_position_msgs.data[0]-increment_angle) ){
				  last_round = true;
			  }else{
				  break;
			  }
		  }
		  
	  }// while no marker found
  }
  if( !ros::ok() ){
	  clean_up();
	  return -1;
  }
  
  // wait for detected markers
  unsigned int nr_of_avg;
  int tmp_nr_of_avg;
  private_ns_node_handle.param( "nr_of_avg", tmp_nr_of_avg, 20);
  if( tmp_nr_of_avg<=0 ){	  
	ROS_WARN("Unexpected nr of averaging samples %i, use default value (%i) insted", nr_of_avg, 20);
	nr_of_avg = 20;
  }else{
	  nr_of_avg = tmp_nr_of_avg;
  }
  
  markers.clear();
  nr_of_samples_received = 0;
  while( nr_of_samples_received<=nr_of_avg && ros::ok() ){
	  ROS_INFO("Buffer marker poses %i/%i received", nr_of_samples_received, nr_of_avg);
	  rate_1s.sleep();
	  ros::spinOnce();
  }
  if( !ros::ok() ){
	  clean_up();
	  return -1;
  }
  
  // disable ar track alvar node
  ar_parameters.request.config.bools[0].value=false;
  ar_set_parameters_client.call( ar_parameters);
  
  // evaluate values
  ROS_INFO("Evaluate values");
  geometry_msgs::Pose estimation;
  if( !markers.localize_base_frame(estimation) ){
	  ROS_ERROR("localization failed");
	  clean_up();
	  return -1;
  }
  
  ROS_INFO("finished localization");
  ROS_INFO("position = [%f, %f, %f]", estimation.position.x, estimation.position.y, estimation.position.z);
  double roll, pitch, yaw;
  tf::Matrix3x3( tf::Quaternion(estimation.orientation.x, estimation.orientation.y, estimation.orientation.z, estimation.orientation.w)).getRPY(roll, pitch, yaw);
  ROS_INFO("orientation = [%f, %f, %f, %f] (quaternion)", estimation.orientation.x, estimation.orientation.y, estimation.orientation.z, estimation.orientation.w);
  ROS_INFO("orientation = [%f, %f, %f] (RPY in degree)", roll*180.0/M_PI, pitch*180.0/M_PI, yaw*180.0/M_PI);
  
  return 0;
  
}

