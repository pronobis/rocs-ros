#ifndef SHOW_TOPO_MAP
#define SHOW_TOPO_MAP

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


class ShowTopoMap{

private:

public:

  rocs_topological_mapping::PlaceHolders placeholders_;
  visualization_msgs::Marker nodes_;
  visualization_msgs::Marker edges_;
  visualization_msgs::Marker doors_;

  visualization_msgs::Marker area1_;
  visualization_msgs::Marker area2_;
  visualization_msgs::Marker area3_;
  visualization_msgs::Marker area4_;
  visualization_msgs::Marker area5_;
  visualization_msgs::Marker area6_;
  visualization_msgs::Marker area7_;
  visualization_msgs::Marker area8_;
  visualization_msgs::Marker area9_;
  visualization_msgs::Marker area10_;
  visualization_msgs::Marker area11_;
  visualization_msgs::Marker area12_;
  visualization_msgs::Marker area13_;

  std::ofstream myfile;



  visualization_msgs::Marker edges_2;

  visualization_msgs::Marker marker;


  ros::Publisher marker_pub;
  ros::Subscriber  place_holders_info;

  ShowTopoMap(ros::NodeHandle &n);

  void infoCallback(const rocs_topological_mapping::PlaceHolders& msg);
  void visualizeNodes ();
  void visualizeEdges ();


protected:
  ros::NodeHandle n_;


};

#endif //TOPOLOGICAL_GRAPH_BUILDER
