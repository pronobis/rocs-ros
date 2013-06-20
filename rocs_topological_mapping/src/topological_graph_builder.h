#ifndef TOPOLOGICAL_GRAPH_BUILDER
#define TOPOLOGICAL_GRAPH_BUILDER

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

#include "place_holder.h"
#include "rocs_topological_mapping/PlaceHolders.h"  //Message
#include "show_topo_map.h"
#include "door_detection/InDoorOpeningDetector.h"
#include "sensor_msgs/LaserScan.h"




class TopologicalGraphBuilder{

private:

public:

  bool inside_room;

  bool inside_door;

  bool leaving_room;

  std::vector<place_holder> place_holders_;
  int number_place_holders_;

  geometry_msgs::Pose robot_current_pose_;
  sensor_msgs::LaserScan laser_scan_;

    
  ros::Subscriber odometry_info_; 
  ros::Publisher place_holders_info_pub_;
  ros::Subscriber  scan_info_;

    
  tf::TransformListener listener;


  TopologicalGraphBuilder(ros::NodeHandle &n);


  void odomCallback(const nav_msgs::OdometryConstPtr& odom);
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);


  bool getCurrentPose(const nav_msgs::OdometryConstPtr& odom);
  int getCurrentNodeId();
  int getCurrentAreaId();


  void addNewPlaceHolder();
  void deleteLastPlaceHolder();
  void addNewDoor();


  float distanceBetweenHoldersAndCurrentPosition();
  float distanceBetweenDoorsAndCurrentPosition();
  void showInformationPlaceHolders();
  void publishInformationPlaceHolders();

  int current_area_set;
  int past_area, past_area_set;
  int next_area;
  int current_area;
  int last_node_id;

  int number_of_areas;


  bool door_founded;


  InDoorOpeningDetector door_detector;




protected:
  ros::NodeHandle n_;


};

#endif //TOPOLOGICAL_GRAPH_BUILDER
