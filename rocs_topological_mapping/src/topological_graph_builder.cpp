/**
 * @file topological_graph_builder
 * @brief Build the topological map based on the position of the robot obteined from odometric information
 * @author Xavier Gallart
 */

#include "topological_graph_builder.h"

bool SAVE_INTO_FILE = false;

std::ofstream myfile;

TopologicalGraphBuilder::TopologicalGraphBuilder(ros::NodeHandle &n): n_(n)
{
  number_place_holders_=0;
  odometry_info_= n_.subscribe("odom", 100, &TopologicalGraphBuilder::odomCallback, this);
  place_holders_info_pub_ = n_.advertise<rocs_topological_mapping::PlaceHolders>("place_holders_info", 1000);

  scan_info_ =n_.subscribe<sensor_msgs::LaserScan>("base_scan", 1000, &TopologicalGraphBuilder::laserCallback, this);

  number_of_areas=1;
  past_area=-1;
  current_area_set=1;
  current_area=1;
  past_area=1;

  inside_room=false;
  inside_door=false;
}

/*!
 * Laser Callback. Update laser_scan_
 */
void TopologicalGraphBuilder::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  //ROS_DEBUG("LaserCallback");
  laser_scan_.angle_increment = msg.get()->angle_increment;
  laser_scan_.angle_max = msg.get()->angle_max;
  laser_scan_.angle_min = msg.get()->angle_min ;
  laser_scan_.header = msg.get()->header ;
  laser_scan_.intensities = msg.get()->intensities ;
  laser_scan_.ranges = msg.get()->ranges ;
  laser_scan_.range_max = msg.get()->range_min ;
  laser_scan_.scan_time = msg.get()->scan_time ;
  laser_scan_.time_increment = msg.get()->time_increment ;
  laser_scan_.__connection_header = msg.get()->__connection_header ;
}


/*!
 * Odometry Callback.
 */

void TopologicalGraphBuilder::odomCallback(const nav_msgs::OdometryConstPtr& odom){

//  ROS_DEBUG("OdomCallback");
//  robot_current_pose_.position.x=odom->pose.pose.position.x;
//  robot_current_pose_.position.y=odom->pose.pose.position.y;
  //~ pos_yaw=tf::getYaw(odom->pose.pose.orientation);

  getCurrentPose(odom);

//  showInformationPlaceHolders();
  publishInformationPlaceHolders();

  current_area=getCurrentAreaId();

  if(current_area != past_area)
    past_area=current_area;


  ROS_DEBUG("Current Area= %d Past area = %d Current set: %d Past set: %d", current_area, past_area, current_area_set, past_area_set);
  ROS_DEBUG("Inside door: %s  Inside room: %s  Last node %d",(inside_door)?"true":"false", (inside_room)?"true":"false", last_node_id) ;


  door_detector.setMinRange(0.15);

  //We found a door
  if(door_detector.inDoorOpening(laser_scan_, 0.6, 1.2) && inside_door==false && door_detector.m_RangeL < 0.55 && door_detector.m_RangeR < 0.55 )
  {

    ROS_DEBUG("Door found. Width: %G \t AngleR: %G\t AngleL: %G\t RangeR: %G\t RangeL: %G\t",
              door_detector.m_Width, door_detector.m_AngleR, door_detector.m_AngleL, door_detector.m_RangeR, door_detector.m_RangeL);

    addNewDoor();

    inside_door = true;
    ROS_DEBUG("START DOOR");


  }
  else
    addNewPlaceHolder();

  //We exit the door
  if(!door_detector.inDoorOpening(laser_scan_, 0.6, 1.2) && inside_door == true)
  {

    inside_door = false;
    ROS_DEBUG("END DOOR");

    if(last_node_id!=-1)     //If the last node is not a door is because we start to leave the room and go to the corridor
    {
      current_area_set=1;   //Corridor area
      inside_room=false;

    }

  }


}


/*!
 *
 */

void TopologicalGraphBuilder::deleteLastPlaceHolder()
{

//  ROS_ERROR("DELETED NODE Number: %d", number_place_holders_);
  number_place_holders_=number_place_holders_-1;
  place_holders_.resize(number_place_holders_);
}


/*!
 *
 */
void TopologicalGraphBuilder::addNewDoor()
{

  if(distanceBetweenDoorsAndCurrentPosition()>1.5 || number_place_holders_==0 )   //Compare between other door to not add 2 doors in the same place
  {
    if(distanceBetweenHoldersAndCurrentPosition()<0.2)    //No have a node so close to the door or even in more or less the same place
    {
      deleteLastPlaceHolder();
    }

    place_holders_.resize(number_place_holders_+1);
    place_holders_[number_place_holders_].setPosition(robot_current_pose_.position);
    place_holders_[number_place_holders_].setIsDoor(1);
    place_holders_[number_place_holders_].setAreaId(-1);
    ROS_DEBUG("Added DOOR, array pose = %d", number_place_holders_);
    number_place_holders_=number_place_holders_+1;

    ROS_ERROR("Number of areas ++");
    number_of_areas++;
    current_area_set=number_of_areas;

    last_node_id = -1;


    if(inside_room==false)
    {
      inside_room=true;
    }
  }
}


/*!
 *
 */
float TopologicalGraphBuilder::distanceBetweenDoorsAndCurrentPosition()
{
  float distance, minimum_distance=100;

  bool founded_door=false;

  for(int i=0;i<number_place_holders_;i++)
  {
    if(!place_holders_[i].door)
    {
      continue;
    }

    founded_door=true;

    distance=sqrt(pow((place_holders_[i].place_holder_position_.x- robot_current_pose_.position.x),2)
                  + pow((place_holders_[i].place_holder_position_.y- robot_current_pose_.position.y),2));

//    ROS_DEBUG("Distance between Actual and DOOR->[%d]=[%f]", i, distance);

    if(i==0 || distance<minimum_distance)
      minimum_distance=distance;
  }

  if(!founded_door)
    minimum_distance=10;    //It is the first door, so we have to add it

//  ROS_DEBUG("Minimum distance door = [%f]", minimum_distance);

  return(minimum_distance);

}

/*!
 *
 */

void TopologicalGraphBuilder::addNewPlaceHolder()
{

//  ROS_DEBUG("Number place holders: [%d]",number_place_holders_);

  if(distanceBetweenHoldersAndCurrentPosition()>1.0 || number_place_holders_==0 )
  {
    place_holders_.resize(number_place_holders_+1);
    place_holders_[number_place_holders_].setPosition(robot_current_pose_.position);
    place_holders_[number_place_holders_].setIsDoor(0); //It is no a door
    place_holders_[number_place_holders_].setAreaId(current_area_set); //Set area id

    ROS_DEBUG("Added place holder number [%d]",number_place_holders_);

    number_place_holders_=number_place_holders_+1;

    past_area_set = current_area_set;

    last_node_id = number_place_holders_+1;

  }
}

/*!
 * \brief Calculate the minimum distance between the current position and all the place holders
 * \return Minimum distance
 */

float TopologicalGraphBuilder::distanceBetweenHoldersAndCurrentPosition()
{
  float distance, minimum_distance;

  for(int i=0;i<number_place_holders_;i++)
  {
    distance=sqrt(pow((place_holders_[i].place_holder_position_.x- robot_current_pose_.position.x),2)
                  + pow((place_holders_[i].place_holder_position_.y- robot_current_pose_.position.y),2));

    ROS_DEBUG("Distance between Actual->[%d]=[%f]", i, distance);

    if(i==0 || distance<minimum_distance)
      minimum_distance=distance;
  }
  
  ROS_DEBUG("Minimum distance = [%f]", minimum_distance);

  return(minimum_distance);
}

/*!
 * Return Node ID of the closest node
 */

int TopologicalGraphBuilder::getCurrentNodeId()
{
  float distance, minimum_distance;
  int node_id=-1;

  for(int i=0;i<number_place_holders_;i++)
  {
    distance=sqrt(pow((place_holders_[i].place_holder_position_.x- robot_current_pose_.position.x),2)
                  + pow((place_holders_[i].place_holder_position_.y- robot_current_pose_.position.y),2));

    ROS_DEBUG("Distance between Actual->[%d]=[%f]", i, distance);

    if(i==0 || distance<minimum_distance)
    {
      minimum_distance=distance;
      node_id=i;
    }
  }

//  ROS_DEBUG("Current node = %d", node_id);

  return(node_id);
}

/*!
 * Return Area ID of the current position
 */

int TopologicalGraphBuilder::getCurrentAreaId()
{
  float distance, minimum_distance;
  int area_id=100;

  for(int i=0;i<number_place_holders_;i++)
  {
    distance=sqrt(pow((place_holders_[i].place_holder_position_.x- robot_current_pose_.position.x),2)
                  + pow((place_holders_[i].place_holder_position_.y- robot_current_pose_.position.y),2));

    ROS_DEBUG("Distance between Actual->[%d]=[%f]", i, distance);

    if(i==0 || distance<minimum_distance)
    {
      minimum_distance=distance;
      if(place_holders_[i].area_id != -1)
        area_id = place_holders_[i].area_id;
    }
  }

//  ROS_DEBUG("Current Area function id = %d", area_id);

  return(area_id);
}


/*!
 *
 */

bool TopologicalGraphBuilder::getCurrentPose(const nav_msgs::OdometryConstPtr& odom){


  tf::StampedTransform transform;
  try{
    listener.waitForTransform("/map", "/base_link",ros::Time(0), ros::Duration(0.1));
    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Error looking up transformation\n%s",ex.what());
  }


  robot_current_pose_.position.x= transform.getOrigin().x();
  robot_current_pose_.position.y= transform.getOrigin().y();
  double yaw_angle= tf::getYaw( transform.getRotation() );

  if(SAVE_INTO_FILE)
  {
    //Copy into a file the estimated pose for all the frames
    myfile << odom->header.seq + 4 << " "
           << odom->header.stamp << " 0 "
           << transform.getOrigin().x() << " "
           << transform.getOrigin().y() << " "
           << transform.getOrigin().z() << " "
           << yaw_angle << std::endl;

  //  ROS_DEBUG("Write file %d  %lf, %lf \n", odom->header.seq, robot_current_pose_.position.x, robot_current_pose_.position.y);

  }



  return true;
}


/*!
 * \brief Show the information of all the place holders
 */

void TopologicalGraphBuilder::showInformationPlaceHolders()
{
  for(int i=0;i<number_place_holders_;i++)
  {
    ROS_DEBUG("Place holder number [%d]. Position: x=[%f] y=[%f]", i, place_holders_[i].place_holder_position_.x,
             place_holders_[i].place_holder_position_.y);

  }
}


/*!
 * \brief Publish into a ROS topic the place information
 */

void TopologicalGraphBuilder::publishInformationPlaceHolders()
{
  rocs_topological_mapping::PlaceHolders msg;

  msg.num_place_holders=number_place_holders_;

//  msg.is_a_door.resize(number_place_holders_);

  for(int i=0;i<number_place_holders_;i++)
  {
    geometry_msgs::Pose pose_place_holder;

    pose_place_holder.position.x=place_holders_[i].place_holder_position_.x;
    pose_place_holder.position.y=place_holders_[i].place_holder_position_.y;

    msg.is_a_door.push_back(place_holders_[i].door);
    msg.area_id.push_back(place_holders_[i].area_id);

    msg.pose.push_back(pose_place_holder);
  }

  place_holders_info_pub_.publish(msg);

}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "rocs_topological_graph_builder");
  ros::NodeHandle n;

  TopologicalGraphBuilder topo_map_builder(n);
  ShowTopoMap show_topo_map(n);

  if(SAVE_INTO_FILE)
    myfile.open ("/home/semmap/Desktop/position_info.dat", std::ios::out);

  ros::Rate r(4);

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();

  }
  
  return(0);
}
