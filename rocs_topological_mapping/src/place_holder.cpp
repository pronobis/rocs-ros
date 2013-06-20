#include "place_holder.h"

void place_holder::setPosition(geometry_msgs::Point pose){
	place_holder_position_.x=pose.x;
	place_holder_position_.y=pose.y;
}

void place_holder::setIsDoor(int isdoor)
{
  door=isdoor;
}

void place_holder::setAreaId(int id)
{
  area_id=id;
}


void place_holder::showPosition(){

  ROS_INFO("Place holder position info: x=[%f] y=[%f]", place_holder_position_.x, place_holder_position_.y);

}


