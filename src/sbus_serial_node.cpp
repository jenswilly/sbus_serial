/*
 * Copyright 2024 Jens Willy Johannsen <jens@jwrobotics.com>, JW Robotics
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
 * SBUS serial node
 *
 * Publishes:
 *	/sbus
 */

#include "rclcpp/rclcpp.hpp"
#include "sbus_serial_driver.h"
#include "sbus_serial/msg/sbus.hpp"
#include <algorithm>
#include <boost/algorithm/clamp.hpp>

class SbusSerial : public rclcpp::Node
{
public:
	SbusSerial() : Node("sbus_serial")
	{
		// Read/set parameters
		this->declare_parameter("port", "/dev/tty11");
		this->declare_parameter("refresh_rate_hz", 2);
		this->declare_parameter("rxMinValue", 172);
		this->declare_parameter("rxMaxValue", 1811);
		this->declare_parameter("outMinValue", 0);
		this->declare_parameter("outMaxValue", 255);
		this->declare_parameter("silentOnFailsafe", false);

		// Parameters for "enable channel". If channel number is -1, no enable channel is used.
		this->declare_parameter("enableChannelNum", -1);
		this->declare_parameter("enableChannelProportionalMin", -1.0);
		this->declare_parameter("enableChannelProportionalMax", -1.0);

		// Deadband is in raw values and is both plus and minus around the midpoint and is used for all channels
		this->declare_parameter("deadband", 0);

		int refresh_rate_hz; // Only used locally for scheduling the timer
		this->get_parameter("port", port_);
		this->get_parameter("refresh_rate_hz", refresh_rate_hz);
		this->get_parameter("rxMinValue", rxMinValue_);
		this->get_parameter("rxMaxValue", rxMaxValue_);
		this->get_parameter("outMinValue", outMinValue_);
		this->get_parameter("outMaxValue", outMaxValue_);
		this->get_parameter("silentOnFailsafe", silentOnFailsafe_);
		this->get_parameter("enableChannelNum", enableChannelNum_);
		this->get_parameter("enableChannelProportionalMin", enableChannelProportionalMin_);
		this->get_parameter("enableChannelProportionalMax", enableChannelProportionalMax_);
		this->get_parameter("deadband", deadband_);

		// Used for mapping raw values
		rawSpan_ = static_cast<float>(rxMaxValue_ - rxMinValue_);
		outSpan_ = static_cast<float>(outMaxValue_ - outMinValue_);

		// Create publisher. Topic is "/sbus" and data type is "sbus_serial::Sbus"
		pub_ = this->create_publisher<sbus_serial::msg::Sbus>("sbus", 10);

		// Initialize SBUS port
		try
		{
			sbusPort_ = new sbus_serial::SBusSerialPort(port_, true);
		}
		catch (...)
		{
			// TODO: add error message in exception and report
			RCLCPP_ERROR(this->get_logger(), "Unable to initalize SBUS port");
			throw; // Re-throw exception
		}

		// Set callback (auto-capture by reference, we need to initialize the timestamp only)
		sbusMsg_ = sbus_serial::msg::Sbus();
		auto now = this->get_clock()->now(); // Get current time from the node's clock. Source must be this->getClock() in order to compare when we receive new messages
		sbusMsg_.header.stamp = now;
		lastPublishedTimestamp_ = now; // Set to same as sbusMsg_.header.stamp so we can see when a new sample is available
		auto callback = [&](const sbus_serial::SBusMsg receivedSbusMsg)
		{
			// Note: sbus_serial::SBusMsg is not the same as sbus_serial::msg::Sbus
			// First check if we should be silent on failsafe and failsafe is set. If so, do nothing
			if (silentOnFailsafe_ && receivedSbusMsg.failsafe)
				return;

			// Next check if we have an "enable channel" specified. If so, return immediately if the value of the specified channel is outside of the specified min/max
			if (enableChannelNum_ >= 1 && enableChannelNum_ <= 16)
			{
				double enableChannelProportionalValue = (receivedSbusMsg.channels[enableChannelNum_ - 1] - rxMinValue_) / rawSpan_;
				if (enableChannelProportionalValue < enableChannelProportionalMin_ || enableChannelProportionalValue > enableChannelProportionalMax_)
					return;
			}

			sbusMsg_.header.stamp = this->get_clock()->now();
			sbusMsg_.frame_lost = receivedSbusMsg.frame_lost;
			sbusMsg_.failsafe = receivedSbusMsg.failsafe;

			// Assign raw channels
			std::transform(receivedSbusMsg.channels.begin(), receivedSbusMsg.channels.end(), sbusMsg_.raw_channels.begin(), [&](uint16_t rawChannel)
						   {
							   return boost::algorithm::clamp(rawChannel, rxMinValue_, rxMaxValue_); // Clamp to min/max raw values
						   });

			// Map to min/max values
			std::transform(receivedSbusMsg.channels.begin(), receivedSbusMsg.channels.end(), sbusMsg_.mapped_channels.begin(), [&](uint16_t rawChannel)
						   {
							   // Set to center output if within deadband
							   int16_t center = (rxMaxValue_ + rxMinValue_) / 2;
							   bool inDeadband = rawChannel >= center - deadband_ && rawChannel <= center + deadband_;
							   if (inDeadband)
								   return (int16_t)(0.5 * outSpan_ + outMinValue_);

							   int16_t mappedValue = (rawChannel - rxMinValue_) / rawSpan_ * outSpan_ + outMinValue_;
							   return boost::algorithm::clamp(mappedValue, outMinValue_, outMaxValue_); // Clamp to min/max output values
						   });
		};
		sbusPort_->setCallback(callback);

		// Schedule timer for periodic publishing
		timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / refresh_rate_hz),
										 std::bind(&SbusSerial::timerCallback, this));

		RCLCPP_INFO(this->get_logger(), "SBUS node started, publisher created, timer scheduled...");
	}

private:
	rclcpp::Publisher<sbus_serial::msg::Sbus>::SharedPtr pub_;
	sbus_serial::SBusSerialPort *sbusPort_;
	sbus_serial::msg::Sbus sbusMsg_;
	std::string port_;
	int rxMinValue_;
	int rxMaxValue_;
	int outMinValue_;
	int outMaxValue_;
	int deadband_;
	bool silentOnFailsafe_;
	int enableChannelNum_;
	double enableChannelProportionalMin_;
	double enableChannelProportionalMax_;
	float rawSpan_;
	float outSpan_;
	rclcpp::TimerBase::SharedPtr timer_; // Periodic timer for publishing at defined rate
	rclcpp::Time lastPublishedTimestamp_;

	void timerCallback()
	{
		// Only publish if we have a new sample
		if (lastPublishedTimestamp_ != sbusMsg_.header.stamp)
		{
			pub_->publish(sbusMsg_);
			lastPublishedTimestamp_ = sbusMsg_.header.stamp;
		}
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SbusSerial>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
