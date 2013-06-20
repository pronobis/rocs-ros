#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Point.h>


class place_holder{
private:

public:

  geometry_msgs::Point place_holder_position_;
  int door;
  int area_id;
 
  void setPosition(geometry_msgs::Point pose);
  void setIsDoor(int isdoor);
  void setAreaId(int area_id);


  void showPosition();

};
