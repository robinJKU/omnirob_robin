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
  
	Object(std::string name, std::string color, std::vector <int> RGBcolor){
	  this->name = name;
	  this->size.resize(3);

	  this->primitive_type.clear();
	  this->primitive_size.clear();
	  this->primitive_position.clear();
	  this->primitive_orientation.clear();

	  this->color = color;
	  this->RGBcolor = RGBcolor;
	  Object::computeHSVcolor(this->HSVcolor, RGBcolor);
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

	std::vector <int> getHSV(){
	  return HSVcolor;
	}

	void printObject(){
	  printf("Object name = %s \n", name.c_str());
	  printf("Object color = %s \n", color.c_str());

	  for(int i = 0; i < primitive_type.size(); i++){
	    printPrimitive(i);
	  }
	}


	void computeHSVcolor(std::vector <int> HSVcolor, std::vector <int> RGBcolor){
	  double r = RGBcolor[0];
	  double g = RGBcolor[1];
	  double b = RGBcolor[2];

	  double max = std::max(r,std::max(g,b));
		double min = std::min(r,std::min(g,b));

		double h = 0;
		if(max == r){
			h = 60.0*(0.0+(g-b)/(max-min));
		}
		if(max == g){
			h = 60.0*(2.0+(b-r)/(max-min));
		}
		if(max == b){
			h = 60.0*(4.0+(r-g)/(max-min));
		}
		if(h < 0){
			h += 360.0;
		}

	  double s = 0;
	  if(max == 0){
	    s = 0;
	  } else {
	    s = (max - min) / max;
	  }

	  double v = max;

	  HSVcolor.push_back(h);
	  HSVcolor.push_back(s);
	  HSVcolor.push_back(v);
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
    
    //color variables
    std::string color;
    std::vector <int> RGBcolor;
    std::vector <int> HSVcolor;   
    
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








