#ifndef __OMNIROB_GEOMETRY_TOOLS_H
#define __OMNIROB_GEOMETRY_TOOLS_H

// geometry_msgs
#include <geometry_msgs/Pose.h>

// tf
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

namespace omnirob_geometry_tools{
	float norm( geometry_msgs::Quaternion quat ){
		return sqrt( quat.x*quat.x + quat.y*quat.y + quat.z*quat.z + quat.w*quat.w);
	}// norm
	
	bool quaternion_is_valid( geometry_msgs::Quaternion quat ){
		return fabs(norm( quat)-1.0)<1e-3;
	}// quaternion is valid
	
	bool pose_is_valid( geometry_msgs::Pose pose ){
		return quaternion_is_valid( pose.orientation);
	}// pose is valid
	
	/**
	 * This function computes the minimal distance between two angles (angle1-angle2) respecting the periodically nature of the angles.
	 * @param angle1: has to be specified in rad
	 * @param angle2: has to be specified in rad 
	 * @return distance_value: it is guanteed that the distance lies in the intervall [-pi,+pi]
	 */
	float min_angle_distance( const float angle1, const float angle2){
		float distance = angle1-angle2;
		
		return fmod( distance+M_PI, 2.0*M_PI) - M_PI;
		
	}// min distance angle
	
	/**
	 * This function computes the maximum of the absolut values of the componentwise differences
	 * @see min_angle_distance.
	 */
	float max_angle_distance( const std::vector<double> &angle_vec1, const std::vector<double> &angle_vec2 ){
		// check input
		if( angle_vec1.size()==0 || angle_vec1.size()!=angle_vec2.size() ){
			ROS_ERROR("Can't compute max angle distance - size of angle vectors is invalid: angle_vec1.size()=%f, angle_vec2.size()=%f", (double) angle_vec1.size(), (double) angle_vec2.size());
			return -1.0;
		}
		
		float max_value = std::numeric_limits<float>::min();
		float distance_ii;
		for( unsigned int element_ii=0; element_ii<angle_vec1.size(); element_ii++){
			distance_ii = fabs( min_angle_distance( angle_vec1[element_ii], angle_vec2[element_ii]));
			
			if( max_value<distance_ii )
				max_value = distance_ii;
				
		}// for all elements
		
		return max_value;
		
	}// max angle distance
	
	float max_angle_distance( const std::vector<float> angle_vec1, const std::vector<float> angle_vec2 ){
		std::vector<double> angle_vec1_float( angle_vec1.begin(), angle_vec1.end());
		std::vector<double> angle_vec2_float( angle_vec2.begin(), angle_vec2.end());
		
		return (float) max_angle_distance( angle_vec1_float, angle_vec2_float);
		
	}// max angle distance
	
	class pose_transformer{
		public:
			/**
			* constructor
			*/
			pose_transformer()
			{
				transform_listener_ = new tf::TransformListener();
			}
			/**
			* destructor
			*/
			~pose_transformer()
			{
				if( transform_listener_!=NULL )
					delete transform_listener_;
			}
			
		public:

			/**
			 * Applys transformation onto pose
			 */
			geometry_msgs::Pose transform_pose( const geometry_msgs::Pose pose, tf::Transform transformation ){
				tf::Transform pose_tf;
				tf::poseMsgToTF( pose, pose_tf); 
				
				tf::Transform pose_transformed_tf = transformation * pose_tf;
				
				geometry_msgs::Pose pose_transformed;
				tf::poseTFToMsg( pose_transformed_tf, pose_transformed);
				
				return pose_transformed;
				
			}// transform pose
		
			/**
			 * Transforms pose describet in original_frame into target_frame
			 */
			geometry_msgs::Pose transform_pose( const geometry_msgs::Pose pose, const std::string original_frame, const std::string target_frame ){
				geometry_msgs::Pose empty_pose;
				
				// check input
				if( !pose_is_valid(pose) ){
					ROS_ERROR("Received invalid pose, can't transform pose");
					return empty_pose;
				}
				if( original_frame.empty() || target_frame.empty() ){
					ROS_ERROR("Invalid frame (original frame %s, target fame %s), can't transform pose", original_frame.c_str(), target_frame.c_str());
					return empty_pose;
				}
				
				// lookup transform
				tf::StampedTransform to_target_frame_from_original_frame_tf;
				try{
					transform_listener_->lookupTransform( target_frame, original_frame, ros::Time(0), to_target_frame_from_original_frame_tf);
					transform_listener_->lookupTransform( target_frame, original_frame, ros::Time(0), to_target_frame_from_original_frame_tf);
				}
				catch (tf::TransformException ex){
					ROS_ERROR("Can't lookup transformation, can't transform pose. Reason: %s",ex.what());
					return empty_pose;
				}
				
				// transform pose
				return transform_pose( pose, to_target_frame_from_original_frame_tf );
				
			}// transform pose
			
		protected:
			tf::TransformListener *transform_listener_;
	};// pose transformer class

	tf::Transform calc_tf( geometry_msgs::Pose pose)
	{
		tf::Transform pose_tf;
		tf::poseMsgToTF( pose, pose_tf);
		return pose_tf;
	}

}

#endif
