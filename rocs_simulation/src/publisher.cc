/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010. David Feil-Seifer
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main( int argc, char* argv[] )
{
  ros::init(argc, argv, "p2os_publisher" );
  ros::NodeHandle n;

  ros::Publisher joint_state_publisher = n.advertise<sensor_msgs::JointState>("joint_states",1000);
  ros::Rate loop_rate(15);
  sensor_msgs::JointState js;

	js.name.push_back(std::string("base_swivel_joint"));
	js.position.push_back(0);
	js.name.push_back(std::string("swivel_hubcap_joint"));
	js.position.push_back(0);
	js.name.push_back(std::string("base_left_wheel_joint"));
	js.position.push_back(0);
	js.name.push_back(std::string("base_right_wheel_joint"));
	js.position.push_back(0);



	while( n.ok() )
	{
    js.header.frame_id="/base_link";
    js.header.stamp = ros::Time::now();
		joint_state_publisher.publish(js);
		ros::spinOnce();
		loop_rate.sleep();
	}
}
