/*
* Copyright 2018 Jens Willy Johannsen <jens@jwrobotics.com>, JW Robotics
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*
* SBUS -> cmd_vel node
* Subscribes to:
*	/sbus
* Publishes:
*	/output/sbus/cmd_vel
*/

#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback( const std_msgs::String::ConstPtr& msg )
{
	ROS_INFO( "I heard: [%s]", msg->data.c_str());
}

int main( int argc, char **argv )
{
	ros::init( argc, argv, "sbus_cmd_vel_node" );
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe( "chatter", 1000, chatterCallback );

	ros::spin();

	return 0;
}
