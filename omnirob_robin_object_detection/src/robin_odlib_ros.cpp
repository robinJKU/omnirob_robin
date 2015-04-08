#include <robin_odlib_ros.h>
#include <ros/ros.h>
#include <sstream>

void robin_odlib_ros::loadObjects(std::vector <Object>& objects){
  
  std::vector <std::string> object_names;
  ros::param::get("/detectable_objects", object_names);
  for(int i = 0; i < object_names.size(); i++){  
    //create the variables for the constructor + param namespace 
    std::string name = object_names[i];
    std::string name_space = "/objects/" + name;
    std::string color;
    std::vector <int> RGBcolor;    
    
    //print name + get the color and RGBcolor value
    ros::param::get(name_space + "/color", color);
    ros::param::get(name_space + "/RGBcolor", RGBcolor);
    int size = RGBcolor.size();
    
    //construct a new object and push it in our vector
    objects.push_back(Object(name, color, RGBcolor));
    
    //now load all the primitives
    int k = 0;
    std::ostringstream s;
    s << k;
    std::string  primitive_space = name_space + "/primitive" + s.str();   
    while(ros::param::has(primitive_space + "_type")) {
      //set the name space for our primitive;      
      
      
      //primitive variables
      std::string type;
      std::vector <double> size;
      std::vector <double> position;
      std::vector <double> orientation;
      ros::param::get(primitive_space + + "_type", type);
      ros::param::get(primitive_space + + "_size", size);
      ros::param::get(primitive_space + + "_position", position);
      ros::param::get(primitive_space + + "_orientation", orientation);
      
      objects[i].addPrimitive(type, size, position, orientation);     
      
      k++;
      s.str(std::string());
      s << k;
      primitive_space = name_space + "/primitive" + s.str();            
    }
    
    objects[i].printObject();   
    
  }  
  
  int size = objects.size();
  printf("\n");
  printf("%d objects loaded \n", size);
  
}
