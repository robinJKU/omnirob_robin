#ifndef SHAPE_H
#define SHAPE_H

#include <string>
#include <vector>

//class Shape holds the complete description for each searchable Shape
class Object{
  public:
  
    Object(std::string name, std::string color, std::vector <int> RGBcolor); 
    
    
    //functions
    static void computeHSVcolor(std::vector <int> HSVcolor, std::vector <int> RGBcolor);  
    void addPrimitive(std::string type, std::vector <double> size, std::vector <double> position, std::vector <double> orientation);
    
    
    //get Values
    std::string getName();
    std::vector <double> getPosition();
    std::vector <double> getOrientation();
    std::vector <double> getSize();
    
    //set Values
    void setPose(std::vector <double> pose);
    void setSize(std::vector <double> size);
    
    
    void printObject();
  
    
  private:
    std::string name;
    std::vector <double> size;
    
    //object definition made from primitives box and cylinder
    std::vector <std::string> primitive_type;    
    std::vector <std::vector <double> > primitive_size;
    std::vector <std::vector <double> > primitive_position;
    std::vector <std::vector <double> > primitive_orientation;
    
    //color variables
    std::string color;
    std::vector <int> RGBcolor;
    std::vector <int> HSVcolor;
        
    //detection variables
    bool detected;
    std::vector <double> position;
    std::vector <double> orientation; 
    
    //functions
    void printPrimitive(int index);
       
	
};
#endif




