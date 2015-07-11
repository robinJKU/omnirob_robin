#include <ros/ros.h>
#include <sstream> 
#include <fstream>

// custom libraries
#include <omnirob_robin_tools_ros/ros_tools.h>
#include <omnirob_robin_driver/driver_tools.h>

// extern libraries
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
#include <omnirob_robin_msgs/localization.h>

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
	
	unsigned int size( void ){
		return samples.size();
	}// size 
	
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
	 * constructor
	 */
	AR_Marker( ):avg_pose_is_initialized(false){}
	
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

/**
 * \class AR_Marker_Localization
 *
 * \brief main class for ar_marker based localization
 *
 */
class AR_Marker_Localization{
	public:
	std::string base_link; /**< base linke which will be detected */
	
	private:
	// handles
	ros::NodeHandle node_handle;
	ros::NodeHandle private_ns_node_handle;
	
	// services, topics and tf
	tf::TransformListener *transform_listener;
	ros::ServiceServer localization_service;
	
	// member variables
	std::vector<AR_Marker> mark; /**< detected marks */
	unsigned int last_considered_id; /**< last id which which were looked for in get_index */
	unsigned int nr_of_samples_received;
	bool detection_mode;
	unsigned int nr_of_markers_received;
        ar_track_alvar_msgs::AlvarMarkers latest_marker;
	Pan_Tilt *pan_tilt;
	
	public:
	/**
	 * constructor: validate and read required paramertes
	 */
	AR_Marker_Localization ():
	last_considered_id(0), node_handle(), private_ns_node_handle("~")
	{
	  // parameter: base_link
	  private_ns_node_handle.getParam("base_link", base_link);
	  if( base_link.empty() ){
		  ROS_INFO("Use default base frame: /base_link");
		  base_link = "/base_link";
	  }
	  
	  // parameter: marker_ids
	  if( !private_ns_node_handle.hasParam("marker_ids") ){
		  ROS_ERROR("No marker ids specifies!");
		  throw -1;
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
		  throw -1;
	  }
	  
	  set_marker_ids( marker_ids);
	  
	  // parameter: marker positions in inertial (=map) frame
	  for( unsigned int marker_ii=0; marker_ii<marker_ids.size(); marker_ii++ ){
		if(    !private_ns_node_handle.hasParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/position/x")
			|| !private_ns_node_handle.hasParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/position/y")
			|| !private_ns_node_handle.hasParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/position/z")
			|| !private_ns_node_handle.hasParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/orientation/x")
			|| !private_ns_node_handle.hasParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/orientation/y")
			|| !private_ns_node_handle.hasParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/orientation/z")
			|| !private_ns_node_handle.hasParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/orientation/w") ){
			ROS_ERROR("Not enought parameters for marker: %s", unsigned_int_to_string(marker_ids[marker_ii]).c_str());
			throw "Not enought parameters for marker";
			
		}else{
			double rx,ry,rz, ox,oy,oz,ow;
			private_ns_node_handle.getParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/position/x",rx);
			private_ns_node_handle.getParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/position/y",ry);
			private_ns_node_handle.getParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/position/z",rz);
			private_ns_node_handle.getParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/orientation/x",ox);
			private_ns_node_handle.getParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/orientation/y",oy);
			private_ns_node_handle.getParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/orientation/z",oz);
			private_ns_node_handle.getParam("marker"+ unsigned_int_to_string(marker_ids[marker_ii]) +"/orientation/w",ow);
			
			mark[ marker_ii].pose_inertial_frame.setOrigin( tf::Vector3(rx,ry,rz));
			mark[ marker_ii].pose_inertial_frame.setRotation( tf::Quaternion(ox,oy,oz,ow));
		}
	  }
	  
	  // construct pan tilt object
	  try{
		pan_tilt = new Pan_Tilt();
	  }catch( char* error_message){
		  throw error_message;
	  }
	  
	  // construct transform listener
	  transform_listener = new tf::TransformListener(node_handle );
	  
	}// constructor
	
	/**
	 * destructor
	 */
	~AR_Marker_Localization( ){
		if( transform_listener!=NULL ){
			delete transform_listener;
		}
		if( pan_tilt!=NULL ){
			delete pan_tilt;
		}
	}// destructor
	
	/**
	 * Localizes the base_link frame through the received poses of ar landmarks. The algorithm build the average over
	 * all (one per landmark) estimated poses. @see Pose_Samples::calc_average
	 * @param estimation: Returns the estimated pose described in map frame
	 * @return Success
	 */
	bool localize_base_frame( geometry_msgs::Pose &estimation ){
		ROS_INFO("localize base link");
		Pose_Samples base_link_poses;
		
		for( unsigned int mark_ii=0; mark_ii<mark.size(); mark_ii++){
			if( mark[mark_ii].marker_detected ){
				ROS_INFO("use data of marker %u whith %i samples", mark[mark_ii].marker_id, mark[mark_ii].observed_pose_reference_frame.size());
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
		ROS_INFO("calc average of localization with %u marker", base_link_poses.size());
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
		
	}// localize_base_frame
	
	/**
	 * @return Index of marker_id id in mark array
	 */
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
	
	/**
	 * @return true if the mark array contains the specified marker_id
	 */
	bool contains( unsigned int marker_id ){
		return this->get_index( marker_id)>=0? true : false;
	}// contains
	
	
	private:
	/**
	 * This function parse the raw_msg and removes those markers which aren't known or are simply too close at the borders.
	 * The latter is important because at the borders ar track alvar returns invalid marker positions.
	 * @param later_than: If lather_than is specified, also a filtering with respect to time is done. This means all messages which are before later_than are discareded. 
	 */
	ar_track_alvar_msgs::AlvarMarkers filter( ar_track_alvar_msgs::AlvarMarkers raw_msg, ros::Time later_than = ros::Time() ){
		ar_track_alvar_msgs::AlvarMarkers marker;
		if( (!later_than.is_zero()) && raw_msg.markers.size()>0 && (raw_msg.markers[0].header.stamp<=later_than) ){
			ROS_INFO("Discard old message");
			return marker;
		}
		
		tf::StampedTransform to_optical_frame_from_ref_frame;
		tf::Transform marker_pose_reference_frame, marker_pose_optical_frame;
		double alpha, beta;
		const static double alpha_max_abs=(53.0-5.0)*M_PI/180.0/2.0;
		const static double beta_max_abs=(71.0-5.0)*M_PI/180.0/2.0;
		
		for( unsigned int marker_ii=0; marker_ii<raw_msg.markers.size(); marker_ii++){
		  // check if marker is known
		  if( !contains( raw_msg.markers[marker_ii].id) ){
			  ROS_WARN("Found unknown marker with id %i", raw_msg.markers[marker_ii].id);
			  continue;
		  }
		  
		  // check if marker is in sight field
		  try{
			  transform_listener->lookupTransform( "/kinect2_rgb_optical_frame", raw_msg.markers[marker_ii].header.frame_id, ros::Time(0), to_optical_frame_from_ref_frame);
			  
		  }catch(tf::TransformException ex){
			   ROS_ERROR("%s",ex.what());
			   continue;
		  }
		  tf::poseMsgToTF( raw_msg.markers[marker_ii].pose.pose, marker_pose_reference_frame);
		  marker_pose_optical_frame = to_optical_frame_from_ref_frame * marker_pose_reference_frame;
		  alpha = atan( marker_pose_optical_frame.getOrigin()[1]/marker_pose_optical_frame.getOrigin()[2]);
		  beta = atan( marker_pose_optical_frame.getOrigin()[0]/marker_pose_optical_frame.getOrigin()[2]);
		  
		  if( fabs(alpha)>alpha_max_abs || fabs(beta)>beta_max_abs ){
			  ROS_WARN("Marker %i is out of sight, discard marker pose", raw_msg.markers[marker_ii].id);
			  continue;
		  }
		  
		  // add marker
		  marker.markers.push_back( raw_msg.markers[marker_ii]);
		
		}
		return marker;
		
	}// filter
	
	/** 
	 * Callback function for ar pose markers
	 */
	void ar_pose_marker_callback( ar_track_alvar_msgs::AlvarMarkers raw_marker_msg ){
		latest_marker = raw_marker_msg;

		ar_track_alvar_msgs::AlvarMarkers marker_msg;
		marker_msg = filter( raw_marker_msg);
		
		unsigned int nr_of_valid_markers = marker_msg.markers.size();
		if( nr_of_valid_markers>0 ){
			if( detection_mode ){
				nr_of_markers_received=nr_of_valid_markers>nr_of_markers_received?nr_of_valid_markers:nr_of_markers_received;
				
			}else{
				for(unsigned int marker_ii=0; marker_ii<marker_msg.markers.size(); marker_ii++ ){
					// if marker id is known, insert marker pose 
					insert_pose( marker_msg.markers[marker_ii].id, marker_msg.markers[marker_ii].pose.pose, marker_msg.markers[marker_ii].header.frame_id);
					
				}// for all markeres
				nr_of_samples_received++;
			}
		}
	}// ar pose marker callback

	bool block_until_latest_marker_is_later_than( ros::Time later_than, float time_out=10.0){

		ros::Rate sleep_rate(2.0);
		if( time_out>=0.0 ){
			sleep_rate = ros::Rate(10.0/time_out);
		}
		unsigned int cnt=0;
		while( ros::ok() && cnt<10.0 && latest_marker.markers.size()>0 && latest_marker.markers[0].header.stamp<=later_than ){
			cnt++;
			sleep_rate.sleep();
			ros::spinOnce();
		}

		return latest_marker.header.stamp<=later_than;

        }// block until latest marker is later than

	/**
	 * Add the specified sample marker_pose to the mark array
	 * @param marker_pose: Pose of marker with id marker_id described in reference_frame_id
	 */
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
	
	public:
	/**
	 * Initializes the mark object by setting the marker_ids
	 */
	void set_marker_ids( std::vector<unsigned int> marker_ids ){
		mark.resize( marker_ids.size());
		for( unsigned marker_ii=0; marker_ii< marker_ids.size(); marker_ii++){
			mark[marker_ii].marker_id = marker_ids[marker_ii];
		}
	}// set marker_ids
	
	/**
	 * Callback of the localization service
	 */
	bool localization_service_callback( omnirob_robin_msgs::localization::Request &req, omnirob_robin_msgs::localization::Response &res ){
		ROS_INFO("Start marker localization -------------------");
		// wait for services
	    std::string pan_tilt_start_motion_srv = "/omnirob_robin/pan_tilt/control/start_motion";
	    if( !omnirob_ros_tools::wait_for_service( pan_tilt_start_motion_srv) ){
		    res.error_message = "Service "+ pan_tilt_start_motion_srv +" is not available";
		    ROS_ERROR("%s", res.error_message.c_str());
		    return true;
	    }
	    std::string ar_track_set_param_srv = "/ar_track_alvar/set_parameters";
	    if( !omnirob_ros_tools::wait_for_service( ar_track_set_param_srv) ){
		    res.error_message = "Service "+ ar_track_set_param_srv +" is not available";
		    ROS_ERROR("%s", res.error_message.c_str());
		    return true;
	    }
	    
	    // enable ar track alvar node
		ros::ServiceClient ar_set_parameters_client = node_handle.serviceClient<dynamic_reconfigure::Reconfigure>( ar_track_set_param_srv);
		dynamic_reconfigure::Reconfigure ar_parameters;
		ar_parameters.request.config.bools.resize(1);
		ar_parameters.request.config.bools[0].name="enabled";
		ar_parameters.request.config.bools[0].value=true;
		ar_set_parameters_client.call( ar_parameters);

		// subscribe to marker poses
		ros::Subscriber ar_pose_marker_subscriber = node_handle.subscribe("/ar_pose_marker", 50, &AR_Marker_Localization::ar_pose_marker_callback, this);

		// look around, search for pan angle overlooking the maximum number of markers
		detection_mode = true;

		double min_angle=-1.8, max_angle=1.8, increment_angle=20.0/180.0*M_PI;
		std::vector<float> pan_tilt_target_position(2);
		pan_tilt_target_position[0] = min_angle;
		
		unsigned int max_nr_of_marks_detected=0;
		double angle_overlooking_maximum_nr_of_markers;
		
		while( ros::ok() && pan_tilt_target_position[0]<=max_angle ){
			// go to next pan angle
			if( !pan_tilt->move_to_state_blocking( pan_tilt_target_position, 10.0) ){
				res.error_message = "Pan Tilt target position isn't reached in time";
				ROS_ERROR("%s", res.error_message.c_str());
				return true;
			}
			
			// check nr of found markers
			block_until_latest_marker_is_later_than( ros::Time::now(), 5.0 );
			nr_of_markers_received=0;
			ros::Rate(1.0).sleep();
			ros::spinOnce();
			if( max_nr_of_marks_detected<nr_of_markers_received ){
				max_nr_of_marks_detected = nr_of_markers_received;
				angle_overlooking_maximum_nr_of_markers = pan_tilt_target_position[0];
			}
			ROS_INFO("detected %i markers at %f degree", nr_of_markers_received, pan_tilt_target_position[0]*180.0/M_PI);
			if( max_nr_of_marks_detected== 1){
				break;
			}
			
			// adapt pan angle
			pan_tilt_target_position[0] += increment_angle;
			
		}// while no marker found
		if( !ros::ok() ){
		  return true;
		}
		if( max_nr_of_marks_detected==0 ){
			res.error_message = "Finished scan, no marker detected";
			return true;
		}
		
		// move to optimal pan angle
		ROS_INFO("move to %f degree", angle_overlooking_maximum_nr_of_markers*180.0/M_PI);
		pan_tilt_target_position[0] = angle_overlooking_maximum_nr_of_markers;
		if( !pan_tilt->move_to_state_blocking( pan_tilt_target_position, 10.0) ){
			res.error_message = "Pan Tilt target position isn't reached in time";
			ROS_ERROR("%s", res.error_message.c_str());
			return true;
		}
	    detection_mode = false;

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

		clear();
		block_until_latest_marker_is_later_than( ros::Time::now(), 5.0 );
		nr_of_samples_received = 0;
		while( nr_of_samples_received<=nr_of_avg && ros::ok() ){
		  ROS_INFO("Buffer marker poses %i/%i received", nr_of_samples_received, nr_of_avg);
		  ros::Rate(1.0).sleep();
		  ros::spinOnce();
		}
		if( !ros::ok() ){
		  return true;
		}

		// disable ar track alvar node
		ar_parameters.request.config.bools[0].value=false;
		ar_set_parameters_client.call( ar_parameters);

		// evaluate values
		ROS_INFO("Evaluate values");
		geometry_msgs::Pose estimation;
		if( !localize_base_frame(estimation) ){
		  res.error_message = "localization failed";
		  ROS_ERROR("%s", res.error_message.c_str());
		  return true;
		}
		
		ROS_INFO("finished localization-----------------------------------");
		ROS_INFO("position = [%f, %f, %f]", estimation.position.x, estimation.position.y, estimation.position.z);
		double roll, pitch, yaw;
		tf::Matrix3x3( tf::Quaternion(estimation.orientation.x, estimation.orientation.y, estimation.orientation.z, estimation.orientation.w)).getRPY(roll, pitch, yaw);
		ROS_INFO("orientation = [%f, %f, %f, %f] (quaternion)", estimation.orientation.x, estimation.orientation.y, estimation.orientation.z, estimation.orientation.w);
		ROS_INFO("orientation = [%f, %f, %f] (RPY in degree)", roll*180.0/M_PI, pitch*180.0/M_PI, yaw*180.0/M_PI);
  
	    res.base_link = estimation;
	    return true;
	    
		
	}// localization service callback
	
	/**
	 * Initiaizes the service object.
	 */
	void advertise_ar_localization_service( void ){
		ROS_INFO("Advertising marker localization service");
		localization_service = node_handle.advertiseService("/marker_localization", &AR_Marker_Localization::localization_service_callback, this);
	}// advertise ar localization service
	
	/**
	 * Reset marker. Remove old samples
	 */
	void clear( void ){
		for( unsigned int mark_ii=0; mark_ii<mark.size(); mark_ii++){
			mark[mark_ii].clear();
		}
		last_considered_id = 0;
	}// clear
	
	/**
	 * Print markers to string
	 */
	std::string to_string( void ){
		std::stringstream string_stream;
		string_stream << "total nr of marker" << mark.size() <<"\n";
		for(unsigned int mark_ii=0; mark_ii<mark.size(); mark_ii++){
			string_stream << "  [" << mark_ii << "]: " << mark[mark_ii].to_string() << "\n";
		}
		return string_stream.str();
	}// to string stream
	
};


std::string unsigned_int_to_string( unsigned int value ){
	std::stringstream value_string_stream;
	value_string_stream << value;
	return value_string_stream.str();
}// unsigned int to string


int main( int argc, char** argv) {
	 
   // initialize node
  ros::init(argc, argv, "marker_localization_global");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_ns_node_handle("~");
  
  AR_Marker_Localization *ar_localization;
  // initialize localization procedure
  try{
	ar_localization = new AR_Marker_Localization;
  }catch(int e){
	ROS_ERROR("cant initialize localization object");
	return e;
  }
  ROS_INFO("Data read successfully:");
  ROS_INFO("%s", ar_localization->to_string().c_str());
  
  // adevertise services
  ar_localization->advertise_ar_localization_service();
  
  // wait for some work
  while( ros::ok() ){
  	ros::spinOnce();
  	ros::Rate(1).sleep();
  }
  
  // clean up and exit
  delete ar_localization;
  return 0;
  
}


