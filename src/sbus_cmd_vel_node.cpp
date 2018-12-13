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
*
* Subscribes to:
*	/sbus (Sbus)
* Publishes:
*	/output/sbus/cmd_vel (Twist)
*/

#include "ros/ros.h"
#include <sbus_serial/Sbus.h>
#include <geometry_msgs/Twist.h>

// Publisher and parameters are in global scope so callback function can use them
ros::Publisher cmdVelPublisher;
int sbusMinValue;
int sbusMaxValue;
int sbusRange;          // Calculated when reading min/max
double maxSpeed;        // m/sec
double maxTurn;         // radians/sec

void sbusCallback( const sbus_serial::Sbus::ConstPtr& msg )
{
	geometry_msgs::Twist twist;

	// Channel 1 (right stick vertical): forward/backward
	double turn = maxTurn * static_cast<double>(msg->mappedChannels[ 0 ]) / 100.0;
	twist.angular.z = turn;

	// Channel 2 (right stick horizontal): turn left/right
	double fwdSpeed = maxSpeed * static_cast<double>(msg->mappedChannels[ 1 ]) / 100.0;
	twist.linear.x = fwdSpeed;

	cmdVelPublisher.publish( twist );
}

int main( int argc, char **argv )
{
	ros::init( argc, argv, "sbus_cmd_vel_node" );
	ros::NodeHandle nh;
	ros::NodeHandle param_nh( "~" );

	// Read/set parameters
	param_nh.param<int>( "sbusMinValue", sbusMinValue, -1 );
	param_nh.param<int>( "sbusMaxValue", sbusMaxValue, -1 );
	param_nh.param<double>( "maxSpeed", maxSpeed, -1 );
	param_nh.param<double>( "maxTurn", maxTurn, -1 );

	// All parameters _must_ be explicitly specified
	if( sbusMaxValue == sbusMinValue && maxSpeed == -1 ) {
		ROS_ERROR( "Config error: sbusMinValue, sbusMaxValue and maxSpeed parameters must be specified!" );
		return 1;
	}

	sbusRange = sbusMaxValue - sbusMinValue;

	ros::Subscriber sbusSubscriber = nh.subscribe( "/sbus", 1, sbusCallback );
	cmdVelPublisher = nh.advertise<geometry_msgs::Twist>( "/output/sbus/cmd_vel", 1 );

	ROS_INFO( "%s started: min/max input = %d/%d, max speed = %.2f m/s, max turn rate = %.2f radians/s", ros::this_node::getName().c_str(), sbusMinValue, sbusMaxValue, maxSpeed, maxTurn );
	ros::spin();
	return 0;
}
