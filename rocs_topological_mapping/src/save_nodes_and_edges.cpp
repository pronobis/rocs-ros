#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <sstream>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <math.h>
#include <string.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>

#include "rocs_topological_mapping/PlaceHolders.h"


class SaveTopoMap{

private:

public:

  rocs_topological_mapping::PlaceHolders placeholders_;
  std::ofstream nodes_file;
  std::ofstream edges_file;


  ros::Subscriber  place_holders_info;

  SaveTopoMap(ros::NodeHandle &n);

  void infoCallback(const rocs_topological_mapping::PlaceHolders& msg);
  void SaveNodes ();
  void SaveEdges ();


protected:
  ros::NodeHandle n_;

};




SaveTopoMap::SaveTopoMap(ros::NodeHandle &n): n_(n)
{

  place_holders_info =n_.subscribe("place_holders_info", 10, &SaveTopoMap::infoCallback, this);

}


void SaveTopoMap::infoCallback(const rocs_topological_mapping::PlaceHolders& msg)
{
  ROS_INFO("Callback: [%d]", msg.num_place_holders);
  placeholders_=msg;

  SaveNodes();
  SaveEdges();
}


void SaveTopoMap::SaveNodes ()
{
  //Copy into a file
  nodes_file.open ("/home/semmap/Desktop/nodes.txt", std::ios::out);

  for(int i=0;i<placeholders_.num_place_holders;i++)
  {
    nodes_file << i << " "
           << placeholders_.pose[i].position.x << " "
           << placeholders_.pose[i].position.y << " "
           << "0 "
           << placeholders_.area_id.at(i) << std::endl;

  }
}

void SaveTopoMap::SaveEdges ()
{
  geometry_msgs::Point p;
  float distance;

  //Copy into a file
  edges_file.open ("/home/semmap/Desktop/edges.txt", std::ios::out);

  for(int i=0;i<placeholders_.num_place_holders;i++)
  {

    //Check the distance between this point and all the others
    for(int j=0;j<placeholders_.num_place_holders;j++)
    {
      if(j==i) continue; //Not compare with itself

      distance=sqrt(pow((placeholders_.pose[i].position.x - placeholders_.pose[j].position.x),2)
                    + pow((placeholders_.pose[i].position.y - placeholders_.pose[j].position.y),2));

//      ROS_INFO("Distance edges = %f", distance);

      if (distance < 1.2)
      {

          edges_file << i << " "
                 << j << std::endl;

      }
    }
  }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "save_nodes_to_file");
  ros::NodeHandle n;

  SaveTopoMap show_topo_map(n);

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return(0);

}
