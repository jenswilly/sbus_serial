/*
 * Copyright 2018-2024 Jens Willy Johannsen <jens@jwrobotics.com>, JW Robotics
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
 */

/* SBUS -> cmd_vel node
 *
 * Subscribes to:
 *	/sbus (Sbus)
 * Publishes:
 *	/output/sbus/cmd_vel (Twist) - remap as necessary
 * Parameters:
 *  forwardChannelIndx (int): Channel index (not channel number!) for forward/reverse. Default is 1 (channel 2)
 *  turnChannelIndx (int): Channel index for turning. Default is 0 (channel 1)
 *  sbusMinValue (int): Minimum value for SBUS channels. Default is 0
 *  sbusMaxValue (int): Maximum value for SBUS channels. Default is 255
 *  minSpeed (double): Minimum speed in m/sec for output Twist. Default is -1.0
 *  maxSpeed (double): Maximum speed in m/sec for output Twist. Default is 1.0
 *  minTurn (double): Minimum turn rate in radians/sec for output Twist. Default is -1.0
 *  maxTurn (double): Maximum turn rate in radians/sec for output Twist. Default is 1.0
 */

#include "rclcpp/rclcpp.hpp"
#include "sbus_interfaces/msg/sbus.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define FORWARD_CHANNEL_INDX 1 // Channel 2 (elevator)
#define TURN_CHANNEL_INDX 0	   // Channel 1 (ailerons)

class SbusCmdVel : public rclcpp::Node
{
public:
	SbusCmdVel() : Node("sbus_cmd_vel")
	{
		// Read/set parameters
		this->declare_parameter("forwardChannelIndx", FORWARD_CHANNEL_INDX); // Channel index (not channel number!) for forward/reverse
		this->declare_parameter("turnChannelIndx", TURN_CHANNEL_INDX);		 // Channel index for turning
		this->declare_parameter("sbusMinValue", 0);							 // Minimum value for SBUS channels
		this->declare_parameter("sbusMaxValue", 255);						 // Maximum value for SBUS channels
		this->declare_parameter("minSpeed", -1.0);							 // Minimum speed in m/sec for output Twist
		this->declare_parameter("maxSpeed", 1.0);							 // Maximum speed in m/sec for output Twist
		this->declare_parameter("minTurn", -1.0);							 // Minimum turn rate in radians/sec for output Twist
		this->declare_parameter("maxTurn", 1.0);							 // Maximum turn rate in radians/sec for output Twist

		this->get_parameter("forwardChannelIndx", forwardChannelIndx_);
		this->get_parameter("turnChannelIndx", turnChannelIndx_);
		this->get_parameter("sbusMinValue", sbusMinValue_);
		this->get_parameter("sbusMaxValue", sbusMaxValue_);
		this->get_parameter("minSpeed", minSpeed_);
		this->get_parameter("maxSpeed", maxSpeed_);
		this->get_parameter("minTurn", minTurn_);
		this->get_parameter("maxTurn", maxTurn_);

		sbusRange_ = sbusMaxValue_ - sbusMinValue_; // Calculate range once

		sbus_sub_ = this->create_subscription<sbus_interfaces::msg::Sbus>("/sbus", 1, std::bind(&SbusCmdVel::sbusCallback, this, std::placeholders::_1));
		cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/output/sbus/cmd_vel", 10);

		RCLCPP_INFO(this->get_logger(), "%s started: min/max input = %d/%d, max speed = %.2f m/s, max turn rate = %.2f radians/s", this->get_name(), sbusMinValue_, sbusMaxValue_, maxSpeed_, maxTurn_);
	}

private:
	rclcpp::Subscription<sbus_interfaces::msg::Sbus>::SharedPtr sbus_sub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

	int sbusMinValue_;
	int sbusMaxValue_;
	int sbusRange_; // Calculated when reading min/max
	double minSpeed_;
	double maxSpeed_; // m/sec
	double minTurn_;
	double maxTurn_; // radians/sec
	int forwardChannelIndx_;
	int turnChannelIndx_;

	void sbusCallback(const sbus_interfaces::msg::Sbus::SharedPtr msg)
	{
		double proportional;
		geometry_msgs::msg::Twist twist;

		proportional = static_cast<double>(msg->mapped_channels[forwardChannelIndx_] - sbusMinValue_) / sbusRange_;
		double fwdSpeed = minSpeed_ + (maxSpeed_ - minSpeed_) * proportional;
		twist.linear.x = fwdSpeed;

		proportional = static_cast<double>(msg->mapped_channels[turnChannelIndx_] - sbusMinValue_) / sbusRange_;
		double turn = minTurn_ + (maxTurn_ - minTurn_) * proportional;
		twist.angular.z = turn;

		cmd_vel_pub_->publish(twist);
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SbusCmdVel>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}