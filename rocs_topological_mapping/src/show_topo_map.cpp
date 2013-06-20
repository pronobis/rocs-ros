#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <sstream>

#include "show_topo_map.h"





ShowTopoMap::ShowTopoMap(ros::NodeHandle &n): n_(n)
{

  marker_pub = n_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  place_holders_info =n_.subscribe("place_holders_info", 10, &ShowTopoMap::infoCallback, this);

  nodes_.header.frame_id = "/map";
  nodes_.header.stamp = ros::Time::now();
  nodes_.ns = "nodes";
  nodes_.action = visualization_msgs::Marker::ADD;
  nodes_.color.r = 0.0;
  nodes_.color.g = 0.0;
  nodes_.color.b = 1.0;
  nodes_.color.a = 0.5;
  nodes_.pose.orientation.w = 1.0;
  nodes_.scale.x = 0.5;
  nodes_.scale.y = 0.5;
  nodes_.scale.z = 0.05;
  nodes_.type = visualization_msgs::Marker::CYLINDER;

  area2_.header.frame_id = "/map";
  area2_.header.stamp = ros::Time::now();
  area2_.ns = "area2";
  area2_.action = visualization_msgs::Marker::ADD;
  area2_.color.r = 0.0;
  area2_.color.g = 0.0;
  area2_.color.b = 0.0;
  area2_.color.a = 0.5;
  area2_.pose.orientation.w = 1.0;
  area2_.scale.x = 0.5;
  area2_.scale.y = 0.5;
  area2_.scale.z = 0.05;
  area2_.type = visualization_msgs::Marker::CYLINDER;

  area3_.header.frame_id = "/map";
  area3_.header.stamp = ros::Time::now();
  area3_.ns = "area3";
  area3_.action = visualization_msgs::Marker::ADD;
  area3_.color.r = 0.0;
  area3_.color.g = 1.0;
  area3_.color.b = 0.0;
  area3_.color.a = 0.5;
  area3_.pose.orientation.w = 1.0;
  area3_.scale.x = 0.5;
  area3_.scale.y = 0.5;
  area3_.scale.z = 0.05;
  area3_.type = visualization_msgs::Marker::CYLINDER;

  area4_.header.frame_id = "/map";
  area4_.header.stamp = ros::Time::now();
  area4_.ns = "area4";
  area4_.action = visualization_msgs::Marker::ADD;
  area4_.color.r = 0.0;
  area4_.color.g = 1.0;
  area4_.color.b = 1.0;
  area4_.color.a = 0.5;
  area4_.pose.orientation.w = 1.0;
  area4_.scale.x = 0.5;
  area4_.scale.y = 0.5;
  area4_.scale.z = 0.05;
  area4_.type = visualization_msgs::Marker::CYLINDER;

  area5_.header.frame_id = "/map";
  area5_.header.stamp = ros::Time::now();
  area5_.ns = "area5";
  area5_.action = visualization_msgs::Marker::ADD;
  area5_.color.r = 1.0;
  area5_.color.g = 0.0;
  area5_.color.b = 1.0;
  area5_.color.a = 0.5;
  area5_.pose.orientation.w = 1.0;
  area5_.scale.x = 0.5;
  area5_.scale.y = 0.5;
  area5_.scale.z = 0.05;
  area5_.type = visualization_msgs::Marker::CYLINDER;

  area6_.header.frame_id = "/map";
  area6_.header.stamp = ros::Time::now();
  area6_.ns = "area6";
  area6_.action = visualization_msgs::Marker::ADD;
  area6_.color.r = 1.0;
  area6_.color.g = 1.0;
  area6_.color.b = 0.0;
  area6_.color.a = 0.5;
  area6_.pose.orientation.w = 1.0;
  area6_.scale.x = 0.5;
  area6_.scale.y = 0.5;
  area6_.scale.z = 0.05;
  area6_.type = visualization_msgs::Marker::CYLINDER;

  area7_.header.frame_id = "/map";
  area7_.header.stamp = ros::Time::now();
  area7_.ns = "area7";
  area7_.action = visualization_msgs::Marker::ADD;
  area7_.color.r = 0.2;
  area7_.color.g = 0.2;
  area7_.color.b = 0.2;
  area7_.color.a = 0.5;
  area7_.pose.orientation.w = 1.0;
  area7_.scale.x = 0.5;
  area7_.scale.y = 0.5;
  area7_.scale.z = 0.05;
  area7_.type = visualization_msgs::Marker::CYLINDER;

  area8_.header.frame_id = "/map";
  area8_.header.stamp = ros::Time::now();
  area8_.ns = "area8";
  area8_.action = visualization_msgs::Marker::ADD;
  area8_.color.r = 0.0;
  area8_.color.g = 0.2;
  area8_.color.b = 0.2;
  area8_.color.a = 0.5;
  area8_.pose.orientation.w = 1.0;
  area8_.scale.x = 0.5;
  area8_.scale.y = 0.5;
  area8_.scale.z = 0.05;
  area8_.type = visualization_msgs::Marker::CYLINDER;

  area9_.header.frame_id = "/map";
  area9_.header.stamp = ros::Time::now();
  area9_.ns = "area9";
  area9_.action = visualization_msgs::Marker::ADD;
  area9_.color.r = 0.0;
  area9_.color.g = 0.5;
  area9_.color.b = 0.0;
  area9_.color.a = 0.5;
  area9_.pose.orientation.w = 1.0;
  area9_.scale.x = 0.5;
  area9_.scale.y = 0.5;
  area9_.scale.z = 0.05;
  area9_.type = visualization_msgs::Marker::CYLINDER;

  area10_.header.frame_id = "/map";
  area10_.header.stamp = ros::Time::now();
  area10_.ns = "area10";
  area10_.action = visualization_msgs::Marker::ADD;
  area10_.color.r = 0.0;
  area10_.color.g = 0.5;
  area10_.color.b = 0.5;
  area10_.color.a = 0.5;
  area10_.pose.orientation.w = 1.0;
  area10_.scale.x = 0.5;
  area10_.scale.y = 0.5;
  area10_.scale.z = 0.05;
  area10_.type = visualization_msgs::Marker::CYLINDER;

  area11_.header.frame_id = "/map";
  area11_.header.stamp = ros::Time::now();
  area11_.ns = "area11";
  area11_.action = visualization_msgs::Marker::ADD;
  area11_.color.r = 0.5;
  area11_.color.g = 0.0;
  area11_.color.b = 0.0;
  area11_.color.a = 0.5;
  area11_.pose.orientation.w = 1.0;
  area11_.scale.x = 0.5;
  area11_.scale.y = 0.5;
  area11_.scale.z = 0.05;
  area11_.type = visualization_msgs::Marker::CYLINDER;

  area12_.header.frame_id = "/map";
  area12_.header.stamp = ros::Time::now();
  area12_.ns = "area12";
  area12_.action = visualization_msgs::Marker::ADD;
  area12_.color.r = 0.5;
  area12_.color.g = 0.0;
  area12_.color.b = 0.5;
  area12_.color.a = 0.5;
  area12_.pose.orientation.w = 1.0;
  area12_.scale.x = 0.5;
  area12_.scale.y = 0.5;
  area12_.scale.z = 0.05;
  area12_.type = visualization_msgs::Marker::CYLINDER;

  area13_.header.frame_id = "/map";
  area13_.header.stamp = ros::Time::now();
  area13_.ns = "area13";
  area13_.action = visualization_msgs::Marker::ADD;
  area13_.color.r = 0.5;
  area13_.color.g = 0.5;
  area13_.color.b = 0.0;
  area13_.color.a = 0.5;
  area13_.pose.orientation.w = 1.0;
  area13_.scale.x = 0.5;
  area13_.scale.y = 0.5;
  area13_.scale.z = 0.05;
  area13_.type = visualization_msgs::Marker::CYLINDER;



  doors_.header.frame_id = "/map";
  doors_.header.stamp = ros::Time::now();
  doors_.ns = "doors";
  doors_.action = visualization_msgs::Marker::ADD;
  doors_.color.r = 1.0;     //Change color
  doors_.color.g = 0.0;
  doors_.color.b = 0.0;
  doors_.scale.x = 0.2;     //Change dimension
  doors_.scale.y = 0.2;
  doors_.scale.z = 0.05;
  doors_.color.a = 1;
  doors_.type = visualization_msgs::Marker::CYLINDER;

  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "marker_test_arrow_by_points";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05; //Diametre eje
  marker.scale.y = 0.05;  //Diametre cap
  marker.scale.z = 0;  //Head lenght
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

}


void ShowTopoMap::infoCallback(const rocs_topological_mapping::PlaceHolders& msg)
{
//  ROS_INFO("Callback: [%d]", msg.num_place_holders);
  placeholders_=msg;

  visualizeNodes();
  visualizeEdges();
}


void ShowTopoMap::visualizeNodes ()
{

  for(int i=0;i<placeholders_.num_place_holders;i++)
  {
//    ROS_INFO("Showing-> Place holder number [%d]. Position: x=[%f] y=[%f]", i, placeholders_.pose[i].position.x,
//             placeholders_.pose[i].position.y);


    geometry_msgs::Point p;
    p.x = placeholders_.pose[i].position.x;
    p.y = placeholders_.pose[i].position.y;
    p.z = 0;

    //Check if it is a door node
    if(placeholders_.is_a_door.at(i)==1)
    {
      doors_.id = i+1;
      doors_.pose.position = p;
      marker_pub.publish(doors_);
    }
    else if(placeholders_.area_id.at(i) == 1)
    {
      nodes_.id = i+1;
      nodes_.pose.position = p;
      marker_pub.publish(nodes_);

    }
    else if(placeholders_.area_id.at(i) == 2)
    {
      area2_.id = i+1;
      area2_.pose.position = p;
      marker_pub.publish(area2_);

    }
    else if(placeholders_.area_id.at(i) == 3)
    {
      area3_.id = i+1;
      area3_.pose.position = p;
      marker_pub.publish(area3_);

    }
    else if(placeholders_.area_id.at(i) == 4)
    {
      area4_.id = i+1;
      area4_.pose.position = p;
      marker_pub.publish(area4_);

    }
    else if(placeholders_.area_id.at(i) == 5)
    {
      area5_.id = i+1;
      area5_.pose.position = p;
      marker_pub.publish(area5_);

    }
    else if(placeholders_.area_id.at(i) == 6)
    {
      area6_.id = i+1;
      area6_.pose.position = p;
      marker_pub.publish(area6_);

    }
    else if(placeholders_.area_id.at(i) == 7)
    {
      area7_.id = i+1;
      area7_.pose.position = p;
      marker_pub.publish(area7_);
    }
    else if(placeholders_.area_id.at(i) == 8)
    {
      area8_.id = i+1;
      area8_.pose.position = p;
      marker_pub.publish(area8_);

    }
    else if(placeholders_.area_id.at(i) == 9)
    {
      area9_.id = i+1;
      area9_.pose.position = p;
      marker_pub.publish(area9_);

    }
    else if(placeholders_.area_id.at(i) == 10)
    {
      area10_.id = i+1;
      area10_.pose.position = p;
      marker_pub.publish(area10_);

    }
    else if(placeholders_.area_id.at(i) == 11)
    {
      area11_.id = i+1;
      area11_.pose.position = p;
      marker_pub.publish(area11_);
    }
    else if(placeholders_.area_id.at(i) == 12)
    {
      area12_.id = i+1;
      area12_.pose.position = p;
      marker_pub.publish(area12_);

    }
    else if(placeholders_.area_id.at(i) == 13)
    {
      area13_.id = i+1;
      area13_.pose.position = p;
      marker_pub.publish(area13_);
    }
  }
}

void ShowTopoMap::visualizeEdges ()
{
  geometry_msgs::Point p;
  float distance;


  int id_marker=1;

  for(int i=0;i<placeholders_.num_place_holders;i++)
  {

    //Check the distance between this point and all the others
    for(int j=0;j<placeholders_.num_place_holders;j++)
    {
      if(j==i) continue; //Not compare with itself

      distance=sqrt(pow((placeholders_.pose[i].position.x - placeholders_.pose[j].position.x),2)
                    + pow((placeholders_.pose[i].position.y - placeholders_.pose[j].position.y),2));

//      ROS_INFO("Distance edges = %f", distance);

      if (distance < 1.3)
      {
        marker.id = id_marker++;

        marker.points.resize(2);

        //Source
        marker.points[0].x = placeholders_.pose[i].position.x;
        marker.points[0].y = placeholders_.pose[i].position.y;
        marker.points[0].z = 0.0f;

        //Destination
        marker.points[1].x = placeholders_.pose[j].position.x;
        marker.points[1].y = placeholders_.pose[j].position.y;
        marker.points[1].z = 0.0f;

        marker_pub.publish(marker);

      }
    }
  }
}

