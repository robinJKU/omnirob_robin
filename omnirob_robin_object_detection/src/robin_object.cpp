#include <robin_object.h>
#include <ros/ros.h>

Object::Object(std::string name, std::string color, std::vector <int> RGBcolor){
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

void Object::addPrimitive(std::string type, std::vector <double> size, std::vector <double> position, std::vector <double> orientation){
  primitive_type.push_back(type);
  primitive_size.push_back(size);
  primitive_position.push_back(position);
  primitive_orientation.push_back(orientation);  
}

std::string Object::getName(){
  return name;
}

std::vector <double> Object::getSize(){
  return size;
}

std::vector <int> Object::getHSV(){
  return HSVcolor;
}

void Object::printObject(){
  printf("Object name = %s \n", name.c_str());
  printf("Object color = %s \n", color.c_str());
  
  for(int i = 0; i < primitive_type.size(); i++){
    printPrimitive(i);
  }  
}

void Object::printPrimitive(int index){
  printf("Primitive %d type = %s \n", index, primitive_type[index].c_str());  
}


void Object::computeHSVcolor(std::vector <int> HSVcolor, std::vector <int> RGBcolor){
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
