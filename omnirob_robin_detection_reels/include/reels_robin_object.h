#ifndef SHAPE_H
#define SHAPE_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <sstream>

class Box{
public:

	Box(){
		size.resize(3);
		position.resize(3);
		orientation.resize(3);
	}

public:
	std::vector<double> size;
	std::vector<double> position;
	std::vector<double> orientation;

	geometry_msgs::Pose get_pose(){

		geometry_msgs::Pose pose;
		pose.position.x = position[0];
		pose.position.y = position[1];
		pose.position.z = position[2];
		tf::Quaternion quat;
		quat.setRPY(orientation[0], orientation[1], orientation[2]);
		quat.normalize();
		tf::quaternionTFToMsg(quat, pose.orientation);
		return pose;
	}


};

//class Shape holds the complete description for each searchable Shape

class Object{
  public:
	Object(){
	}
  
	Object(std::string name){
	  this->name = name;
	  this->size.resize(3);

	  this->primitive_type.clear();
	  this->primitive_size.clear();
	  this->primitive_position.clear();
	  this->primitive_orientation.clear();
	}
    
    
	void addPrimitive(std::string type, std::vector <double> size, std::vector <double> position, std::vector <double> orientation){
	  this->primitive_type.push_back(type);
	  this->primitive_size.push_back(size);
	  this->primitive_position.push_back(position);
	  this->primitive_orientation.push_back(orientation);
	  this->resize();
	}

	Box getBox(){
		return bounding_box;
	}

	void set_number(int number){
		std::stringstream s;
		s << name;
		s << "_";
		s << number;
		name = s.str();
	}


	void setBox(Box input_box){
		bounding_box = input_box;
	}

	tf::Transform get_transform(){
		tf::Transform transform;
		tf::poseMsgToTF(bounding_box.get_pose(), transform);
		return transform;
	}

	std::string getName(){
	  return name;
	}

	std::vector <double> getSize(){
	  return size;
	}


	void printObject(){
	  printf("Object name = %s \n", name.c_str());

	  for(int i = 0; i < primitive_type.size(); i++){
	    printPrimitive(i);
	  }
	}
    
  private:
    std::string name;
    std::vector <double> size;
    Box bounding_box;
    
    //object definition made from primitives box and cylinder
    std::vector <std::string> primitive_type;    
    std::vector <std::vector <double> > primitive_size;
    std::vector <std::vector <double> > primitive_position;
    std::vector <std::vector <double> > primitive_orientation;
    
    
    //functions
    void printPrimitive(int index){
	  printf("Primitive %d type = %s \n", index, primitive_type[index].c_str());
	}

    void resize(){
	  std::vector <double> min;
	  std::vector <double> max;
	  for(int i = 0; i < 3; i++){
		min.push_back(0);
		max.push_back(0);
	  }

	  for(int i = 0; i < primitive_size.size(); i++){
		for(int k = 0; k < 3; k++){
		  double val = -primitive_size[i][k] / 2.0 + primitive_position[i][k];  //ADD ROTATION
		  if(val < min[k]){
			min[k] = val;
		  }
		  val = primitive_size[i][k] / 2.0 + primitive_position[i][k];
		  if(val > max[k]){
			max[k] = val;
		  }
		}
	  }
	  for(int i = 0; i < 3; i++){
		size[i] = max[i] - min[i];
		bounding_box.size[i] = size[i];
	  }
	}
};


class Cylinder{
  public:

	std::vector<double> size;
	std::vector<double> normal;
	std::vector<double> position;
	std::vector<double> orientation;
	std::string name;

	Cylinder(){
		size.resize(3);
		normal.resize(3);
		position.resize(3);
		orientation.resize(3);
	}

  public:

	geometry_msgs::Pose get_pose(){  

		geometry_msgs::Pose pose;
		pose.position.x = position[0];
		pose.position.y = position[1];
		pose.position.z = position[2];
		tf::Quaternion quat;
		quat.setRPY(orientation[0], orientation[1], orientation[2]);
		quat.normalize();
		tf::quaternionTFToMsg(quat, pose.orientation);
		return pose;
	}

	void set_number(int number){

		std::stringstream s;
		s << name;
		s << "_";
		s << number;
		name = s.str();
	}

	std::string getName(){
	  return name;
	}

};


class AR_Marker{

  public:
	unsigned int marker_id;// id of the marker
	std::string reference_frame_id; //Reference frame of the marker
	geometry_msgs::Pose observed_pose;
	tf::Vector3 position;
	bool marker_detected;

  public:
	/**
	 * constructor
	 */

	 AR_Marker( ):marker_detected(false){} //Default constructor

};

#endif








